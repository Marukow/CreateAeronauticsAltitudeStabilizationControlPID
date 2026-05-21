-- pls someone make this better and send it back to me, thank you.
-- i think i'm going mad

-- put a monitor on top of the computer or change the monitor variable
-- also change the MOTOR_ variables to match your quadcopter setup
-- alt_kp, ki and kd need to be tuned
-- stab_kp, ki and kd also need to be tuned
-- try not tunning first, maybe it works, we'll never know

-- CONFIG
local TARGET_ALT = 80

local MOTOR_FL = "Create_RotationSpeedController_0"
local MOTOR_FR = "Create_RotationSpeedController_1"
local MOTOR_BL = "Create_RotationSpeedController_3"
local MOTOR_BR = "Create_RotationSpeedController_2"

local GIMBAL_NAME = "gimbal_sensor_3"

local MONITOR_SIDE = "top"

-- Altitude PID
local ALT_KP = 3.0
local ALT_KI = 0.1
local ALT_KD = 2.0

-- Stabilization PID
local STAB_KP = 0.5
local STAB_KI = 0.0001
local STAB_KD = 0.1

-- Stabilization correction limits
local STAB_CORR_MIN, STAB_CORR_MAX = -80, 80
local STAB_INTEG_MIN, STAB_INTEG_MAX = -20, 20

-- Disturbance detection: when a large sudden tilt spike occurs (plane landing or departing)
local DISTURBANCE_THRESHOLD = 3.0
local INTEGRAL_BLEED        = 0.3

-- Motor limits
local MOTOR_MIN, MOTOR_MAX = -256, 256

-- Altitude correction limits
local ALT_CORR_MIN, ALT_CORR_MAX = -100, 100
local ALT_INTEG_MIN, ALT_INTEG_MAX = -60, 60

-- K: is the lift coefficient that maps motor RPM and air pressure to actual thrust:
--   lift = k * RPM * pressure
local k        = nil
local K_MIN_RPM = 20
local K_ALPHA   = 0.05

-- SETUP
local pid     = require("pid")
local monitor = peripheral.wrap(MONITOR_SIDE)

local motorFL = peripheral.wrap(MOTOR_FL)
local motorFR = peripheral.wrap(MOTOR_FR)
local motorBL = peripheral.wrap(MOTOR_BL)
local motorBR = peripheral.wrap(MOTOR_BR)

-- Gimbal - .getAngles()
-- a[1] --> yaw (+ left, - right)
-- a[2] --> pitch (+ up, - down)
local gimbal = peripheral.wrap(GIMBAL_NAME)

if not motorFL then error("No motor: FL") end
if not motorFR then error("No motor: FR") end
if not motorBL then error("No motor: BL") end
if not motorBR then error("No motor: BR") end
if not monitor then error("No monitor")   end
if not gimbal then error("No gimbal")     end

monitor.setTextScale(0.5)
monitor.clear()

local altPID = pid.new(TARGET_ALT, ALT_KP, ALT_KI, ALT_KD)
altPID:clampOutput(ALT_CORR_MIN, ALT_CORR_MAX)
altPID:limitIntegral(ALT_INTEG_MIN, ALT_INTEG_MAX)

local rollPID  = pid.new(0, STAB_KP, STAB_KI, STAB_KD)
local pitchPID = pid.new(0, STAB_KP, STAB_KI, STAB_KD)
rollPID:limitIntegral(STAB_INTEG_MIN, STAB_INTEG_MAX)
pitchPID:limitIntegral(STAB_INTEG_MIN, STAB_INTEG_MAX)
rollPID:clampOutput(STAB_CORR_MIN, STAB_CORR_MAX)
pitchPID:clampOutput(STAB_CORR_MIN, STAB_CORR_MAX)

-- HELPERS
local function displayLine(row, text)
    monitor.setCursorPos(1, row)
    monitor.clearLine()
    monitor.write(text)
end

-- Clampinsons
local function clamp(val, min, max)
    return math.max(min, math.min(max, val))
end

-- Set motor speeds
local function setMotorSpeeds(fl, fr, bl, br)
    motorFL.setTargetSpeed(clamp(fl, MOTOR_MIN, MOTOR_MAX))
    motorFR.setTargetSpeed(clamp(fr, MOTOR_MIN, MOTOR_MAX))
    motorBL.setTargetSpeed(clamp(bl, MOTOR_MIN, MOTOR_MAX))
    motorBR.setTargetSpeed(clamp(br, MOTOR_MIN, MOTOR_MAX))
end

-- Feedforward calculation
local FALLBACK_C = 61.81
local function getFeedforward(pressure, mass, gravity)
    if pressure == nil or pressure == 0 then return 0 end
    local C
    if k ~= nil then
        C = (mass * gravity) / k
    else
        C = FALLBACK_C
    end
    return C / pressure
end

-- K estimation
local function updateK(mass, gravity, vertAccel, currentRPM, pressure)

    if currentRPM < K_MIN_RPM or pressure == nil or pressure == 0 then return end
    local lift = mass * (gravity + vertAccel)
    if lift <= 0 then return end
    local kNew = lift / (currentRPM * pressure)
    if k == nil then
        k = kNew
    else
        k = k * (1 - K_ALPHA) + kNew * K_ALPHA
    end

end

-- Control loop
local function controlLoop()
    local lastTime = os.clock()
    local prevVelY = 0

    while true do
        local now = os.clock()
        local dt  = math.max(now - lastTime, 0.001)
        lastTime  = now

        -- Taking the data
        local pose     = sublevel.getLogicalPose()
        local pos      = pose.position
        local angVel   = sublevel.getAngularVelocity()
        local pressure = aero.getAirPressure(pos)
        local mass     = sublevel.getMass()
        local gravityVec = aero.getGravity()
        local gravity    = math.abs(gravityVec.y)
        local velocity = sublevel.getLinearVelocity()
        local velY = velocity.y
        local com = sublevel.getCenterOfMass()

        -- Vertical acceleration for k and feedforward
        local vertAccel = (velY - prevVelY) / dt
        prevVelY = velY

        -- Average actual RPM across all four motors.
        local currentRPM = (motorFL.getTargetSpeed() + motorFR.getTargetSpeed()
                          + motorBL.getTargetSpeed() + motorBR.getTargetSpeed()) / 4
        updateK(mass, gravity, vertAccel, currentRPM, pressure)

        -- Altitude
        local ff      = getFeedforward(pressure, mass, gravity)
        local altCorr = altPID:step(pos.y, dt) - ALT_KD * velY
        local baseRPM = ff + altCorr

        -- Stabilization
        local tiltErr = gimbal.getAngles()
        local rollErr = tiltErr[1]
        local pitchErr = tiltErr[2]

        -- If detect large sudden tilt bleed integral
        if math.abs(rollErr) > DISTURBANCE_THRESHOLD then
            rollPID.integral = rollPID.integral * INTEGRAL_BLEED
        end
        if math.abs(pitchErr) > DISTURBANCE_THRESHOLD then
            pitchPID.integral = pitchPID.integral * INTEGRAL_BLEED
        end

        -- PID outputs with derivative damping based on angular velocity
        local rollOutput  = rollPID:step(rollErr, dt)  - STAB_KD * angVel.z
        local pitchOutput = pitchPID:step(pitchErr, dt) - STAB_KD * angVel.x

        rollOutput  = clamp(rollOutput,  STAB_CORR_MIN, STAB_CORR_MAX)
        pitchOutput = clamp(pitchOutput, STAB_CORR_MIN, STAB_CORR_MAX)

        -- Motor mixing ---------------- WARNING: THIS IS PROBABLY NOT CORRECT FOR YOUR AIRCRAFT, TUNE CAREFULLY  ------------- use the little wand thingy to see if the roll and pitch outputs are going in the right direction, if not swap signs or something
        local fl = (baseRPM + pitchOutput) - rollOutput
        local fr = (baseRPM + pitchOutput) + rollOutput
        local bl = (baseRPM - pitchOutput) - rollOutput
        local br = (baseRPM - pitchOutput) + rollOutput

        setMotorSpeeds(fl, fr, bl, br)

        -- Display
        displayLine(1,  "Target: " .. TARGET_ALT .. " m")
        displayLine(2,  string.format("Alt:   %6.2f m",    pos.y))
        displayLine(3,  string.format("Err:  %+6.2f m",    TARGET_ALT - pos.y))
        displayLine(4,  string.format("FF:   %+6.2f rpm",  ff))
        displayLine(5,  string.format("Corr: %+6.2f rpm",  altCorr))
        displayLine(6,  string.format("Base: %+6.2f rpm",  baseRPM))
        displayLine(7,  string.format("Roll: %+6.2f deg / Out: %+5.1f", rollErr, rollOutput))
        displayLine(8,  string.format("Ptch: %+6.2f deg / Out: %+5.1f", pitchErr, pitchOutput))
        displayLine(9,  string.format("FL:%+5.0f FR:%+5.0f", fl, fr))
        displayLine(10, string.format("BL:%+5.0f BR:%+5.0f", bl, br))
        displayLine(11, k and string.format("K:  %.6f", k) or "K:  (warmup)")
        displayLine(13, string.format("CoM: %.2f %.2f %.2f", com.x, com.y, com.z))
        displayLine(14, string.format("Mass: %.2f kg", mass))
        displayLine(15, string.format("Grav: %.2f m/s²", gravity))
        displayLine(16, string.format("Weight: %.2f N", mass * gravity))
        displayLine(17, string.format("Pres: %.2f Pa", pressure))
        displayLine(18, string.format("VelY: %.2f m/s", velY))
        displayLine(19, string.format("VertAccel: %.2f m/s²", vertAccel))

        sleep(0.05)
    end
end

-- User input loop
local function inputLoop()
    while true do
        io.write("New altitude: ")

        local input = read()
        local newAlt = tonumber(input)

        if newAlt then
            TARGET_ALT = newAlt
            altPID.sp  = newAlt
            altPID.integral   = 0
            altPID.prev_error = 0
            print("Target set to " .. newAlt .. " m")
        else
            setMotorSpeeds(0, 0, 0, 0)
            error("Terminated")
        end
    end
end
-- Run both loops in parallel
parallel.waitForAny(controlLoop, inputLoop)