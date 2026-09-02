-- Cart_Cargo_Controller.lua
-- 根据小车位置自动显示/隐藏货物箱体
-- 等待位→显示货物（载料待命），供料位→隐藏货物（已卸料）
-- 长期运行脚本

sim = require('sim')

local CartA, CartB
local A_Supply, B_Supply

-- 获取 cargo 树
local function getCargoTree(cartHandle, prefix)
    local name = prefix .. '_CargoParts'
    -- 搜索 cart 下的 cargo
    local objs = sim.getObjectsInTree(cartHandle, sim.handle_all, 1)
    for _, h in ipairs(objs) do
        local n = ''
        pcall(function() n = sim.getObjectAlias(h, 0) or '' end)
        if n == name then return h end
    end
    return -1
end

local function setTreeVisible(root, visible)
    if root == -1 then return end
    local objs = sim.getObjectsInTree(root, sim.handle_all, 0)
    for _, h in ipairs(objs) do
        sim.setObjectInt32Param(h, sim.objintparam_visibility_layer, visible and 1 or 0)
    end
end

local function isAtPosition(cartHandle, targetHandle)
    if cartHandle == -1 or targetHandle == -1 then return false end
    local cp = sim.getObjectPosition(cartHandle, -1)
    local tp = sim.getObjectPosition(targetHandle, -1)
    local dx = cp[1] - tp[1]
    local dy = cp[2] - tp[2]
    local dz = cp[3] - tp[3]
    return math.sqrt(dx*dx + dy*dy + dz*dz) < 0.01
end

local cargoA = -1
local cargoB = -1

function sysCall_init()
    print('===== Cart Cargo Controller =====')

    CartA = sim.getObject('/CartA')
    CartB = sim.getObject('/CartB')
    A_Supply = sim.getObject('/CartA_SupplyPose')
    B_Supply = sim.getObject('/CartB_SupplyPose')

    if CartA == -1 or CartB == -1 then
        print('[WARN] Carts not found. Run Cart_Cargo_Setup first.')
        return
    end

    -- 查找 cargo 对象
    cargoA = getCargoTree(CartA, 'A')
    cargoB = getCargoTree(CartB, 'B')

    if cargoA == -1 then
        print('[WARN] CartA cargo not found. Run Cart_Cargo_Setup first.')
    end
    if cargoB == -1 then
        print('[WARN] CartB cargo not found. Run Cart_Cargo_Setup first.')
    end

    print('[OK] Cargo controller ready')
end

function sysCall_actuation()
    -- CartA: 在供料位隐藏（已卸料），否则显示（载料中）
    if cargoA ~= -1 then
        setTreeVisible(cargoA, not isAtPosition(CartA, A_Supply))
    end
    -- CartB: 在供料位隐藏（已卸料），否则显示（载料中）
    if cargoB ~= -1 then
        setTreeVisible(cargoB, not isAtPosition(CartB, B_Supply))
    end
end
