-- Cart_Cargo_Setup.lua
-- 在小车上放置真实AB物料模型（复制自实际供料件）
-- CartA 载 A 物料，CartB 载 B 物料
-- 运行一次后禁用

sim = require('sim')

-- 复制一棵对象树到小车顶部
local function copyPartsToCart(sourcePath, cartHandle, prefix)
    if cartHandle == -1 then return nil end

    local source = sim.getObject(sourcePath)
    if source == -1 then
        print('[WARN] Source not found: ' .. sourcePath)
        return nil
    end

    -- 复制整棵树
    local objs = sim.getObjectsInTree(source, sim.handle_all, 0)
    local copy = sim.copyPasteObjects(objs, 0)

    -- 找到复制后的根节点
    local root = nil
    for _, h in ipairs(copy) do
        if sim.getObjectParent(h) == -1 then
            root = h; break
        end
    end

    if not root then return nil end

    -- 改名
    sim.setObjectAlias(root, prefix .. '_CargoParts')

    -- 挂到小车下
    sim.setObjectParent(root, cartHandle, true)

    -- 放到小车顶部（小车高约0.145m，放在上面）
    local cartPos = sim.getObjectPosition(cartHandle, -1)
    local sourcePos = sim.getObjectPosition(source, -1)
    local dx = sourcePos[1] - cartPos[1]
    local dy = sourcePos[2] - cartPos[2]
    local dz = sourcePos[3] - cartPos[3]
    -- 移到小车顶
    sim.setObjectPosition(root, cartHandle, {0, 0, 0.11})

    -- 初始隐藏
    for _, h in ipairs(copy) do
        sim.setObjectInt32Param(h, sim.objintparam_visibility_layer, 0)
    end

    return root
end

function sysCall_init()
    print('===== Cart Cargo Setup (Real Parts) =====')

    local CartA = sim.getObject('/CartA')
    local CartB = sim.getObject('/CartB')

    if CartA == -1 or CartB == -1 then
        print('[ERROR] CartA or CartB not found')
        return
    end

    -- 删除旧的简版cargo（如果存在）
    for _, prefix in ipairs({'A', 'B'}) do
        local old = sim.getObject('/' .. prefix .. '_CargoBox')
        if old ~= -1 then sim.removeObjects(sim.getObjectsInTree(old, sim.handle_all, 0)) end
    end

    -- CartA: 复制A供料件（箱体+PCB+模块+端子排）
    local a1 = copyPartsToCart('/FiveCR5A_Cell/Parts/Box_Blank', CartA, 'A')
    local a2 = copyPartsToCart('/FiveCR5A_Cell/Parts/PCB_Supply', CartA, 'A')
    local a3 = copyPartsToCart('/FiveCR5A_Cell/Parts/Control_Module_Supply', CartA, 'A')
    local a4 = copyPartsToCart('/FiveCR5A_Cell/Parts/Terminal_Block_Supply', CartA, 'A')

    -- 调整A零件在小车上的位置（堆叠）
    if a1 then
        sim.setObjectPosition(a1, CartA, {0, 0, 0.10})        -- 箱体在底部
    end
    if a4 then
        sim.setObjectPosition(a4, CartA, {0.045, -0.035, 0.14}) -- 端子排
    end
    if a2 then
        sim.setObjectPosition(a2, CartA, {0, 0, 0.143})        -- PCB
    end
    if a3 then
        sim.setObjectPosition(a3, CartA, {0.025, 0.025, 0.15}) -- 模块
    end

    if a1 then print('[OK] CartA: A parts loaded') end

    -- CartB: 复制B供料件
    local b1 = copyPartsToCart('/FiveCR5A_Cell/PartsB/Box_Blank_B', CartB, 'B')
    local b2 = copyPartsToCart('/FiveCR5A_Cell/PartsB/PCB_Supply_B', CartB, 'B')
    local b3 = copyPartsToCart('/FiveCR5A_Cell/PartsB/Control_Module_Supply_B', CartB, 'B')
    local b4 = copyPartsToCart('/FiveCR5A_Cell/PartsB/Terminal_Block_Supply_B', CartB, 'B')

    if b1 then
        sim.setObjectPosition(b1, CartB, {0, 0, 0.10})
    end
    if b4 then
        sim.setObjectPosition(b4, CartB, {0.045, -0.035, 0.14})
    end
    if b2 then
        sim.setObjectPosition(b2, CartB, {0, 0, 0.143})
    end
    if b3 then
        sim.setObjectPosition(b3, CartB, {0.025, 0.025, 0.15})
    end

    if b1 then print('[OK] CartB: B parts loaded') end

    print('===== Setup done. =====')
end
