-- Update_Cart_Script.lua
-- 更新场景中 Cart_Order_Link_Manager 的嵌入式脚本为新版（含3秒延迟）
-- 运行一次后禁用

sim = require('sim')

function sysCall_init()
    local newScript = [[
-- Cart_Order_Link_Manager.lua
-- ROS2订单控制小车
-- /product_order:
-- A: CartA->Supply, CartB->Wait
-- B: CartA->Wait, CartB->Supply
-- 到位后等待3秒发送 product_type

sim=require('sim')
simROS2=require('simROS2')

local sub=nil

local CartA=sim.getObject('/CartA')
local CartB=sim.getObject('/CartB')

local CartA_SupplyPose=sim.getObject('/CartA_SupplyPose')
local CartA_WaitPose=sim.getObject('/CartA_WaitPose')
local CartB_SupplyPose=sim.getObject('/CartB_SupplyPose')
local CartB_WaitPose=sim.getObject('/CartB_WaitPose')

local speed=0.2
local tolerance=0.002

local carts={
 A={handle=CartA,target=nil,moving=false,arrived=true},
 B={handle=CartB,target=nil,moving=false,arrived=true}
}

local order=nil
local changing=false
local arriveTime=nil
local SWITCH_DELAY=3.0

local function setTarget(id,target)
    carts[id].target=target
    carts[id].moving=true
    carts[id].arrived=false
end

local function updateCart(id)
    local c=carts[id]
    if not c.moving then return end
    local p=sim.getObjectPosition(c.handle,-1)
    local t=sim.getObjectPosition(c.target,-1)
    local dx=t[1]-p[1]; local dy=t[2]-p[2]; local dz=t[3]-p[3]
    local d=math.sqrt(dx*dx+dy*dy+dz*dz)
    if d<tolerance then
        sim.setObjectPosition(c.handle,-1,t)
        c.moving=false; c.arrived=true
        print("Cart"..id.." arrived")
        return
    end
    local step=speed*sim.getSimulationTimeStep()
    if step>d then step=d end
    sim.setObjectPosition(c.handle,-1,{
        p[1]+dx/d*step, p[2]+dy/d*step, p[3]+dz/d*step})
end

local function allArrived()
    return carts.A.arrived and carts.B.arrived
end

local function orderA()
    order="A"; changing=true; arriveTime=nil
    setTarget("A",CartA_SupplyPose); setTarget("B",CartB_WaitPose)
end

local function orderB()
    order="B"; changing=true; arriveTime=nil
    setTarget("A",CartA_WaitPose); setTarget("B",CartB_SupplyPose)
end

function callback(msg)
    print("Order:",msg.data)
    if msg.data=="A" then orderA()
    elseif msg.data=="B" then orderB() end
end

function sysCall_init()
    sub=simROS2.createSubscription('/product_order','std_msgs/msg/String','callback')
    print("Cart_Order_Link_Manager ready (delay="..SWITCH_DELAY.."s)")
end

function sysCall_actuation()
    updateCart("A"); updateCart("B")
    if changing and allArrived() then
        if arriveTime==nil then
            arriveTime=sim.getSimulationTime()
            print("Carts arrived, waiting "..SWITCH_DELAY.."s...")
        end
        if sim.getSimulationTime()-arriveTime>=SWITCH_DELAY then
            sim.setStringSignal('product_type',order)
            print("product_type:",order)
            changing=false; arriveTime=nil
        end
    end
end

function sysCall_cleanup()
    if sub then simROS2.shutdownSubscription(sub) end
end
]]

    -- 搜索所有 child script，找到 Cart_Order_Link_Manager 的脚本
    local targets = {'XCKZ', 'ABqiehuan', 'ABdis', 'ASPRO', 'script'}
    local updated = 0

    for _, name in ipairs(targets) do
        local h = sim.getObject('/FiveCR5A_Cell/' .. name)
        if h == -1 then h = sim.getObject('/' .. name) end
        if h ~= -1 then
            local s = sim.getScript(sim.scripttype_childscript, h)
            if s ~= -1 then
                local old = ''
                pcall(function() old = sim.getScriptText(s) end)
                if string.find(old, 'Cart_Order_Link_Manager') or string.find(old, 'product_order') then
                    sim.setScriptText(s, newScript)
                    print('✅ Updated: ' .. name .. ' (Cart_Order_Link_Manager)')
                    updated = updated + 1
                end
            end
        end
    end

    -- 也检查场景根脚本
    if updated == 0 then
        -- 遍历所有对象找
        local allObjs = sim.getObjects(sim.handle_all)
        for _, h in ipairs(allObjs) do
            local s = sim.getScript(sim.scripttype_childscript, h)
            if s ~= -1 then
                local old = ''
                pcall(function() old = sim.getScriptText(s) end)
                if string.find(old, 'Cart_Order_Link_Manager') or string.find(old, '/product_order') then
                    sim.setScriptText(s, newScript)
                    local n = ''
                    pcall(function() n = sim.getObjectAlias(h, 0) or '' end)
                    print('✅ Updated: ' .. n .. ' (handle=' .. h .. ')')
                    updated = updated + 1
                end
            end
        end
    end

    if updated == 0 then
        print('❌ Cart_Order_Link_Manager script not found in scene.')
        print('   Please manually update the script from:')
        print('   scenes/Cart_Order_Link_Manager.lua')
    else
        print('===== Update complete. Save scene (Ctrl+S). =====')
    end
end
