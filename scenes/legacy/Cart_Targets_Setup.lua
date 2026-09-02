-- Cart_Targets_Setup.lua
-- 创建小车目标点（供料位 + 等待位），遵循 Step03 模式
-- 使用：新建 Dummy 挂载本脚本，运行一次后禁用
-- 目标点创建在 /FiveCR5A_Cell/Targets/Cart_Targets 下

sim = require('sim')

local RECREATE_TARGETS = true
local SHOW_TARGETS = true

local TARGETS = {
    {name = 'CartA_SupplyPose', pos = {-2.30, -0.90, 0.05}, color = {1.0, 0.5, 0.2}},
    {name = 'CartB_SupplyPose', pos = {-1.80, -0.90, 0.05}, color = {0.2, 0.6, 1.0}},
    {name = 'CartA_WaitPose',   pos = {-2.30, -1.55, 0.05}, color = {1.0, 0.5, 0.2}},
    {name = 'CartB_WaitPose',   pos = {-1.80, -1.55, 0.05}, color = {0.2, 0.6, 1.0}},
}

local function safeGet(path)
    local ok, h = pcall(sim.getObject, path)
    if ok then return h end
    return -1
end

local function colorDummy(h, color)
    pcall(sim.setObjectColor, h, 0, sim.colorcomponent_ambient_diffuse, color)
end

local function removeTree(h)
    if h == -1 then return end
    local ok, objs = pcall(sim.getObjectsInTree, h, sim.handle_all, 0)
    if ok and objs then
        sim.removeObjects(objs)
    else
        sim.removeObjects({h})
    end
end

function sysCall_init()
    print('===== Cart Target Points Setup =====')

    local cell = safeGet('/FiveCR5A_Cell')
    if cell == -1 then
        print('[ERROR] /FiveCR5A_Cell not found. Run Step01 first.')
        return
    end

    -- 确保 Targets 容器存在
    local targets = safeGet('/FiveCR5A_Cell/Targets')
    if targets == -1 then
        targets = sim.createDummy(0.030)
        sim.setObjectAlias(targets, 'Targets')
        sim.setObjectParent(targets, cell, true)
    end

    -- 创建 Cart_Targets 组
    local cartGroup = safeGet('/FiveCR5A_Cell/Targets/Cart_Targets')
    if cartGroup ~= -1 and RECREATE_TARGETS then
        removeTree(cartGroup)
        cartGroup = -1
    end
    if cartGroup == -1 then
        cartGroup = sim.createDummy(0.025)
        sim.setObjectAlias(cartGroup, 'Cart_Targets')
        sim.setObjectParent(cartGroup, targets, true)
        sim.setObjectPosition(cartGroup, -1, {0, 0, 0})
    end

    -- 创建目标点
    for _, t in ipairs(TARGETS) do
        local old = safeGet('/' .. t.name)
        if old ~= -1 and RECREATE_TARGETS then
            removeTree(old)
        end

        if safeGet('/' .. t.name) == -1 then
            local d = sim.createDummy(0.035)
            sim.setObjectAlias(d, t.name)
            sim.setObjectParent(d, cartGroup, true)
            sim.setObjectPosition(d, -1, t.pos)
            sim.setObjectOrientation(d, -1, {0, 0, 0})

            if not SHOW_TARGETS then
                pcall(sim.setObjectInt32Param, d, sim.objintparam_visibility_layer, 0)
            end

            colorDummy(d, t.color)
            print(string.format('[CART TARGET] %s = {%.3f, %.3f, %.3f}', t.name, t.pos[1], t.pos[2], t.pos[3]))
        else
            print('[CART TARGET] ' .. t.name .. ' already exists, skip')
        end
    end

    print('===== Cart Target Points Done =====')
    print('[NEXT] Disable this script. Import Carts as /CartA and /CartB.')
end

function sysCall_cleanup()
end
