local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local option = require("option")
local order = require("util.order")

local Scheduler = require("util.scheduler")
local scheduler = Scheduler.new()
local request = Scheduler.request

local kTasks = {
	robot_status = require("task.robot_status"),
}

local kNavigateTimeout = 30 -- 秒，单段导航超时上限（可用 ROS 参数 train_navigate_timeout 覆盖）
local kNearTolerance = 0.5 -- 米，判定到达的容差（可用 ROS 参数 train_near_tolerance 覆盖）

---
--- Export Context
---

blackboard = require("blackboard").singleton()

on_init = function()
	action:bind(scheduler)
	action:info(ascii.banner)
	action:info("操作手训练......")

	clock:reset(blackboard.meta.timestamp)
	action:update_enable_control(false)

	local train_started = false

	local navigate_timeout = option.train_navigate_timeout or kNavigateTimeout
	local near_tolerance = option.train_near_tolerance or kNearTolerance

	--- @param point { x: number, y: number }
	--- @return boolean reached
	local navigate = function(point)
		action:gimbal_suspend()
		action:navigate(point)
		action:info("navigate to " .. point.x .. ", " .. point.y)

		local timeout = request:wait_until {
			monitor = function()
				return blackboard.condition.near(point, near_tolerance)
			end,
			timeout = navigate_timeout,
		}
		if timeout then
			action:fuck("navigate timeout, current x=" .. blackboard.user.x .. " y=" .. blackboard.user.y)
		end

		return not timeout
	end

	local intent_handler = nil
	local intent = function()
		local initial_pose = nil
		local goal_pose = nil

		while true do
			local found_initial, ix, iy, iyaw = action:consume_initial_pose()
			if found_initial then
				initial_pose = { x = ix, y = iy, yaw = iyaw }
				action:info("set initial pose: " .. ix .. ", " .. iy)
			end

			local found_goal, gx, gy, gyaw = action:consume_goal_pose()
			if found_goal then
				goal_pose = { x = gx, y = gy, yaw = gyaw }
				action:info("set goal pose: " .. gx .. ", " .. gy)
			end

			if initial_pose ~= nil and goal_pose ~= nil then
				action:info("起终点已就绪，开始导航")

				if navigate(initial_pose) then
					navigate(goal_pose)
				end

				-- 停止周期性重发，等待下一对点击
				action:cancel_target()

				initial_pose = nil
				goal_pose = nil
			end

			request:yield()
		end
	end

	local toggle_train = function()
		if train_started then
			action:info("导航关闭")

			action:switch_motion_mode("normal")
			action:gimbal_suspend()

			request:sleep(0.5)
			action:update_enable_control(false)
			action:cancel_target()
			action:switch_topic_forward(false)
			action:switch_pose_capture(false)

			action:update_supercap_boost(false)
			action:stop_navigation()

			if intent_handler ~= nil then
				intent_handler.cancel()
				intent_handler = nil
			end

			train_started = false
		else
			action:info("导航启动")

			-- 先开启点击捕获，避免启动期间点击丢失
			action:switch_topic_forward(false)
			action:switch_pose_capture(true)

			action:restart_navigation {
				global_map = "战队",
				launch_livox = true,
				launch_odin1 = false,
				use_sim_time = false,
			}

			request:sleep(5)
			action:relocalize()
			request:sleep(5)

			action:update_enable_control(true)
			action:switch_motion_mode("normal")
			action:gimbal_suspend()

			intent_handler = scheduler:append_task(intent)

			train_started = true
		end
	end

	scheduler:append_task(kTasks.robot_status)
	scheduler:append_task(function()
		local switch_order = order.new(blackboard.getter.rswitch, 1)
		switch_order:on({ "MIDDLE", "UP", "MIDDLE" }, toggle_train)

		while true do
			switch_order:spin()
			request:yield()
		end
	end)
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)
	scheduler:spin_once()
end

on_exit = function()
	action:cancel_target()
	action:switch_pose_capture(false)
	action:stop_navigation()
end
