--- A basic PID type and common PID operations. This may be useful
-- when working with control systems.
--
-- An introduction to PIDs can be found on [Wikipedia][wiki].
--
-- [wiki]: https://en.wikipedia.org/wiki/Proportional-integral-derivative_controller
--
-- If you are interested in using [CCSharp][ccsharp], here is the compatible [PID.cs][ccsharp-pid] file.
--
-- [ccsharp]: https://github.com/monkeymanboy/CCSharp
-- [ccsharp-pid]: https://github.com/monkeymanboy/CCSharp/blob/master/src/CCSharp/AdvancedMath/PID.cs
--
-- @module pid
-- @author TechTastic

local expect = require "cc.expect"
local expect = expect.expect
local metatable

--- Performs a PID control step if the setpoint is a scalar (number) value
--
-- @tparam pid self The PID instance
-- @tparam number value The current value being measured
-- @tparam number dt The time since the last step
-- @treturn the control output
-- @usage output = pid:step(value)
-- @usage output = pid:step(value, 0.5)
-- @local
local function scalarStep(self, value, dt)
    expect(1, value, "number")
    expect(2, dt, "number", "nil")
    dt = dt or 1

    local error = self.sp - value
    local p = self.kp * error
    if self.discrete then
        self.integral = self.integral + error * dt
    else
        self.integral = self.integral + (error + self.prev_error) * dt * 0.5
    end

    if self.integral_min and self.integral_max then
        self.integral = math.max(self.integral_min, math.min(self.integral_max, self.integral))
    end

    local i = self.ki * self.integral
    local d = self.kd * (error - self.prev_error) / dt
    self.prev_error = error
    
    local output = p + i + d
    if self.output_min and self.output_max then
        return math.max(self.output_min, math.min(self.output_max, output))
    end
    return output
end

--- Performs a PID control step if the setpoint is a vector value
--
-- @tparam pid self The PID instance
-- @tparam number value The current value being measured
-- @tparam number dt The time since the last step
-- @treturn the control output vector
-- @usage output = pid:step(value)
-- @usage output = pid:step(value, 0.5)
-- @local
local function vectorStep(self, value, dt)
    expect(1, value, "table")
    if (getmetatable(value) or {}).__name ~= "vector" then expect(1, value, "vector") end
    expect(2, dt, "number", "nil")
    dt = dt or 1

    local error = self.sp - value
    local p = error * self.kp
    if self.discrete then
        self.integral = self.integral + error * dt
    else
        self.integral = self.integral + (error + self.prev_error) * dt * 0.5
    end

    if self.integral_min and self.integral_max then
        self.integral = vector.new(
            math.max(self.integral_min, math.min(self.integral_max, self.integral.x)),
            math.max(self.integral_min, math.min(self.integral_max, self.integral.y)),
            math.max(self.integral_min, math.min(self.integral_max, self.integral.z))
        )
    end

    local i = self.integral * self.ki
    local d = (error - self.prev_error) * (self.kd / dt)
    self.prev_error = error
    
    local output = p + i + d
    if self.output_min and self.output_max then
        return vector.new(
            math.max(self.output_min, math.min(self.output_max, output.x)),
            math.max(self.output_min, math.min(self.output_max, output.y)),
            math.max(self.output_min, math.min(self.output_max, output.z))
        )
    end
    return output
end

--- Performs a PID control step if the setpoint is a quaternion value
--
-- @tparam pid self The PID instance
-- @tparam number value The current value being measured
-- @tparam number dt The time since the last step
-- @treturn the angular velocity control output
-- @usage output = pid:step(value)
-- @usage output = pid:step(value, 0.5)
-- @local
-- @see quaternion
local function quaternionStep(self, value, dt)
    expect(1, value, "table")
    if (getmetatable(value) or {}).__name ~= "quaternion" then expect(1, value, "quaternion") end
    expect(2, dt, "number", "nil")
    dt = dt or 1

    local error_quat = self.sp * value:inverse()
    local error_vec = error_quat:getAxis() * error_quat:getAngle()
    local p = error_vec * self.kp
    if self.discrete then
        self.integral = self.integral + error_vec * dt
    else
        self.integral = self.integral + (error_vec + self.prev_error) * dt * 0.5
    end

    if self.integral_min and self.integral_max then
        self.integral = vector.new(
            math.max(self.integral_min, math.min(self.integral_max, self.integral.x)),
            math.max(self.integral_min, math.min(self.integral_max, self.integral.y)),
            math.max(self.integral_min, math.min(self.integral_max, self.integral.z))
        )
    end

    local i = self.integral * self.ki
    local d = (error_vec - self.prev_error) * (self.kd / dt)
    self.prev_error = error_vec

    local output = p + i + d
    if self.output_min and self.output_max then
        return vector.new(
            math.max(self.output_min, math.min(self.output_max, output.x)),
            math.max(self.output_min, math.min(self.output_max, output.y)),
            math.max(self.output_min, math.min(self.output_max, output.z))
        )
    end
    return output
end

--- Constructors
--
-- @section Constructors

--- Constructs a new PID controller for either a scalar, vector, or quaternion target.
--
-- @tparam number|vector|quaternion target The setpoint to reach
-- @tparam number p Proportional gain - how aggressively to respond to the current error
-- @tparam number i Integral gain - how aggressively to eliminate accumulated error
-- @tparam number d Derivative gain - how aggressively to dampen the rate of change
-- @tparam boolean discrete Whether to treat the PID as discrete or continuous
-- @treturn The PID initialized with the given arguments
-- @usage pid = pid.new(target)
-- @export
-- @see quaternion
function new(target, p, i, d, discrete)
    expect(1, target, "table", "number")

    local targetMeta = getmetatable(target) or {}
    if type(target) == "table" and targetMeta.__name ~= "vector" and targetMeta.__name ~= "quaternion" then
        expect(1, target, "vector", "quaternion", "number")
    end
    expect(2, p, "number", "nil")
    expect(3, i, "number", "nil")
    expect(4, d, "number", "nil")
    expect(5, discrete, "boolean", "nil")

    local controller = {
        sp = target or 1,
        kp = p or 1,
        ki = i or 0,
        kd = d or 0,
        discrete = discrete or true
    }
    if type(target) == "number" then
        controller.step = scalarStep
        controller.integral = 0
        controller.prev_error = 0
    elseif type(target) == "table" then
        if targetMeta.__name == "vector" then
            controller.step = vectorStep
            controller.integral = vector.new()
            controller.prev_error = vector.new()
        elseif targetMeta.__name == "quaternion" then
            controller.step = quaternionStep
            controller.integral = vector.new()
            controller.prev_error = vector.new()
        end
    end
    return setmetatable(controller, metatable)
end

--- A PID, with a scalar, vector, or quaternion setpoint, kP, kI, and kD, both as discrete and continuous.
--
-- @type PID
local pid = {
    --- The setpoint to reach. Uses [Vector](https://tweaked.cc/module/vector.html) or Quaternion types when applicable.
    -- @field sp
    -- @tparam number|vector|quaternion sp
    -- @see quaternion

    --- The proportional gain - how aggressively to respond to the current error
    -- @field kp
    -- @tparam number kp

    --- The integral gain - how aggressively to eliminate accumulated error
    -- @field ki
    -- @tparam number ki

    --- The derivative gain - how aggressively to dampen the rate of change
    -- @field kd
    -- @tparam number kd

    --- Whether to treat the PID as discrete or continuous
    -- @field discrete
    -- @tparam boolean discrete

    --- Performs a PID control step.
    -- Uses [Vector](https://tweaked.cc/module/vector.html) or Quaternion types when applicable.
    -- @tparam PID self The PID instance
    -- @tparam number|vector|quaternion value The current value being measured
    -- @tparam number dt The time since the last step
    -- @treturn number|vector|quaternion The control output
    -- @usage output = pid:step(value)
    -- @usage output = pid:step(value, 0.5)
    -- @see quaternion
    step = function() end,  -- to be replaced in constructor

    --- Enables/disables the clamping of the output value
    --
    -- @tparam PID self The PID instance
    -- @tparam number min The minimum clamp for the output
    -- @tparam number max The maximum clamp for the output
    clampOutput = function(self, min, max)
        expect(2, min, "number", "nil")
        if min then
            expect(3, max, "number")
        else
            expect(3, max, "nil")
        end
        if min >= max then
            error("Invalid limits! Min must be less than max!")
        end
        self.output_min = min
        self.output_max = max
    end,

    --- Enables/disables the integral limits for anti-windup
    --
    -- @tparam PID self The PID instance
    -- @tparam number min The minimum limit for the integral
    -- @tparam number max The maximum limit for the integral
    limitIntegral = function(self, min, max)
        expect(2, min, "number", "nil")
        if min then
            expect(3, max, "number")
        else
            expect(3, max, "nil")
        end
        if min >= max then
            error("Invalid limits! Min must be less than max!")
        end
        self.integral_min = min
        self.integral_max = max
    end,

    --- Converts the PID instance into a human-readable string
    --
    -- @tparam PID self The PID instance
    -- @treturn string The PID instance as a human-readable string
    tostring = function(self)
        local mode = self.discrete and "Discrete" or "Continuous"
        local sp = tostring(self.sp)
        -- FIX: was using %d for Kp and Kd, which truncates floats like 0.5 → 0.
        -- %g prints the shortest accurate representation of any number.
        return string.format("%s PID {SP = %s, Kp = %g, Ki = %g, Kd = %g}", mode, sp, self.kp, self.ki, self.kd)
    end
}

metatable = {
    __name = "PID",
    __index = pid,
    __tostring = pid.tostring
}

return {new = new}
