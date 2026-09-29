local Map = require("map.core").new()
local action = require("action")
local request = require("util.scheduler").request
local bb = require("blackboard").singleton()

local Points = {
	kOrigin = Map:point("kBegin", { x = 0.0, y = 0.3 }),
	kAttackInLeft = Map:point("kAttackInLeft", { x = 0.0, y = 2.4 }),
	kAttackInRight = Map:point("kAttackInRight", { x = 2.55, y = 0.0 }),
	kStepBegin = Map:point("kStepBegin", { x = 2.5, y = -0.8 }),
	kStepFinal = Map:point("kStepFinal", { x = 2.5, y = -2.2 }),
	kUnderSlope = Map:point("kUnderSlope", { x = 4.7, y = -2.1 }),
	kUnderRune = Map:point("kUnderRune", { x = 4.7, y = -2.1 }),
	kAttackAtHole = Map:point("kAttackAtHole", { x = 4.7, y = -2.1 }),
}

local function navigate(_, to)
	action:navigate(to)
	action:info("navigate to " .. to.x .. ", " .. to.y)
	local timeout = request:wait_until {
		monitor = function()
			return bb.condition.near(to, 0.5)
		end,
		timeout = 15,
	}
	if timeout then
        action:fuck("navigate timeout, current x=" .. bb.user.x .. " y=" .. bb.user.y)
    end
	
	return not timeout
end
Map:connect(Points.kOrigin, Points.kAttackInLeft) { navigate, navigate }
Map:connect(Points.kOrigin, Points.kAttackInRight) { navigate, navigate }
Map:connect(Points.kAttackInRight, Points.kStepBegin) { navigate, navigate }
Map:connect(Points.kStepBegin, Points.kStepFinal) { navigate, navigate }
Map:connect(Points.kStepFinal, Points.kUnderSlope) { navigate, navigate }
Map:connect(Points.kUnderSlope, Points.kUnderRune) { navigate, navigate }
Map:connect(Points.kUnderRune, Points.kAttackAtHole) { navigate, navigate }

return {
	map = Map,
	points = Points,
}
