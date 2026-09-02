-- Cart_Cargo_Full.lua
-- 创建真实外观货物 + 自动显隐，一体化脚本
-- 新建 Dummy → Non-threaded child script → 粘贴 → 保持启用

sim = require('sim')

local CartA, CartB
local A_Supply, B_Supply
local allCargo = {}     -- {handle, cart ('A' or 'B')}

local function makeBox(cart, size, pos, color)
    local h = sim.createPrimitiveShape(sim.primitiveshape_cuboid, size, 0)
    sim.setShapeColor(h, nil, sim.colorcomponent_ambient_diffuse, color)
    sim.setObjectPosition(h, cart, pos)
    sim.setObjectParent(h, cart, true)
    sim.setObjectInt32Param(h, sim.objintparam_visibility_layer, 0)
    return h
end

local function isAt(cart, target)
    if cart == -1 or target == -1 then return false end
    local cp = sim.getObjectPosition(cart, -1)
    local tp = sim.getObjectPosition(target, -1)
    return math.sqrt((cp[1]-tp[1])^2 + (cp[2]-tp[2])^2 + (cp[3]-tp[3])^2) < 0.015
end

function sysCall_init()
    CartA = sim.getObject('/CartA')
    CartB = sim.getObject('/CartB')
    A_Supply = sim.getObject('/CartA_SupplyPose')
    B_Supply = sim.getObject('/CartB_SupplyPose')

    if CartA == -1 or CartB == -1 then
        print('[Cargo] Carts not found')
        return
    end

    -- 只在首次创建（检查 CartA 下是否已有 cargo）
    local existing = sim.getObjectsInTree(CartA, sim.handle_all, 1)
    local hasCargo = false
    for _, h in ipairs(existing) do
        local n = ''
        pcall(function() n = sim.getObjectAlias(h, 0) or '' end)
        if n == 'A_CargoBox' then hasCargo = true; break end
    end

    if not hasCargo then
        print('[Cargo] Creating...')

        -- === CartA: A物料（灰箱+绿PCB+深模块+黄端子） ===
        local a1 = makeBox(CartA, {0.21, 0.15, 0.072}, {0, 0, 0.10}, {0.60, 0.60, 0.60})
        sim.setObjectAlias(a1, 'A_CargoBox')
        table.insert(allCargo, {h=a1, cart='A'})

        local t = makeBox(CartA, {0.19, 0.006, 0.050}, {0, -0.072, 0.11}, {0.70, 0.70, 0.70})
        table.insert(allCargo, {h=t, cart='A'})

        t = makeBox(CartA, {0.14, 0.09, 0.006}, {0, 0.02, 0.142}, {0.0, 0.45, 0.18})
        table.insert(allCargo, {h=t, cart='A'})

        t = makeBox(CartA, {0.04, 0.04, 0.010}, {-0.02, 0.02, 0.148}, {0.10, 0.10, 0.10})
        table.insert(allCargo, {h=t, cart='A'})

        t = makeBox(CartA, {0.05, 0.04, 0.020}, {0.025, 0.020, 0.15}, {0.15, 0.15, 0.20})
        table.insert(allCargo, {h=t, cart='A'})

        t = makeBox(CartA, {0.10, 0.020, 0.020}, {0.045, -0.030, 0.14}, {0.92, 0.82, 0.35})
        table.insert(allCargo, {h=t, cart='A'})

        -- === CartB: B物料（蓝橙箱+底部法兰+紫PCB银芯片+橙红模块+绿端子+条纹） ===
        local b1 = makeBox(CartB, {0.21, 0.15, 0.072}, {0, 0, 0.10}, {0.15, 0.20, 0.55})
        sim.setObjectAlias(b1, 'B_CargoBox')
        table.insert(allCargo, {h=b1, cart='B'})

        t = makeBox(CartB, {0.19, 0.006, 0.050}, {0, -0.072, 0.11}, {0.95, 0.40, 0.10})
        table.insert(allCargo, {h=t, cart='B'})

        t = makeBox(CartB, {0.23, 0.17, 0.006}, {0, 0, 0.062}, {0.12, 0.12, 0.14})
        table.insert(allCargo, {h=t, cart='B'})

        -- 四角加强筋
        for _, c in ipairs({{-0.10,-0.07},{0.10,-0.07},{-0.10,0.07},{0.10,0.07}}) do
            t = makeBox(CartB, {0.016, 0.016, 0.025}, {c[1], c[2], 0.060}, {0.90, 0.45, 0.10})
            table.insert(allCargo, {h=t, cart='B'})
        end

        t = makeBox(CartB, {0.14, 0.09, 0.006}, {0, 0.02, 0.142}, {0.35, 0.05, 0.35})
        table.insert(allCargo, {h=t, cart='B'})

        t = makeBox(CartB, {0.04, 0.04, 0.010}, {-0.02, 0.02, 0.148}, {0.85, 0.85, 0.90})
        table.insert(allCargo, {h=t, cart='B'})

        t = makeBox(CartB, {0.05, 0.04, 0.020}, {0.025, 0.020, 0.15}, {0.95, 0.25, 0.15})
        table.insert(allCargo, {h=t, cart='B'})

        t = makeBox(CartB, {0.05, 0.02, 0.010}, {0.027, 0.020, 0.16}, {1.0, 1.0, 1.0})
        table.insert(allCargo, {h=t, cart='B'})

        t = makeBox(CartB, {0.10, 0.020, 0.020}, {0.045, -0.030, 0.14}, {0.10, 0.70, 0.25})
        table.insert(allCargo, {h=t, cart='B'})

        -- 左右外壁竖条纹
        for j = 1, 3 do
            t = makeBox(CartB, {0.004, 0.015, 0.055}, {-0.107, (j-2)*0.04, 0.10}, {0.08, 0.12, 0.45})
            table.insert(allCargo, {h=t, cart='B'})
            t = makeBox(CartB, {0.004, 0.015, 0.055}, {0.107, (j-2)*0.04, 0.10}, {0.08, 0.12, 0.45})
            table.insert(allCargo, {h=t, cart='B'})
        end

        print('[Cargo] Created ' .. #allCargo .. ' parts')
    else
        -- 收集已有 cargo handles
        local function collectCart(cartHandle, cartId)
            local kids = sim.getObjectsInTree(cartHandle, sim.handle_all, 1)
            for _, h in ipairs(kids) do
                local n = ''
                pcall(function() n = sim.getObjectAlias(h, 0) or '' end)
                if string.find(n, 'Cargo') then
                    local subs = sim.getObjectsInTree(h, sim.handle_all, 0)
                    for _, s in ipairs(subs) do
                        table.insert(allCargo, {h=s, cart=cartId})
                    end
                end
            end
            -- 也收集非 CargoBox 的散件
            for _, h in ipairs(kids) do
                local t = ''
                pcall(function() t = tostring(sim.getObjectType(h)) end)
                if t == tostring(sim.object_shape_type) then
                    local alreadyIn = false
                    for _, c in ipairs(allCargo) do
                        if c.h == h then alreadyIn = true; break end
                    end
                    if not alreadyIn then
                        table.insert(allCargo, {h=h, cart=cartId})
                    end
                end
            end
        end
        collectCart(CartA, 'A')
        collectCart(CartB, 'B')
        print('[Cargo] Found ' .. #allCargo .. ' existing parts')
    end

    print('[Cargo] Ready')
end

function sysCall_actuation()
    if CartA == -1 or CartB == -1 then return end

    local aSupply = isAt(CartA, A_Supply)
    local bSupply = isAt(CartB, B_Supply)

    for _, c in ipairs(allCargo) do
        if c.cart == 'A' then
            sim.setObjectInt32Param(c.h, sim.objintparam_visibility_layer, aSupply and 0 or 1)
        else
            sim.setObjectInt32Param(c.h, sim.objintparam_visibility_layer, bSupply and 0 or 1)
        end
    end
end
