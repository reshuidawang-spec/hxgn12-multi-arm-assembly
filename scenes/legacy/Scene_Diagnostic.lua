-- Scene_Diagnostic.lua
-- 场景功能诊断脚本。在CoppeliaSim中运行一次，输出所有对象/脚本/信号。
-- 运行后把控制台输出完整复制给我。

sim = require('sim')

function sysCall_init()
    print('==================== SCENE DIAGNOSTIC ====================')
    print('')

    -- 1. 场景根结构
    print('--- 1. ROOT OBJECTS ---')
    local ok, objs = pcall(sim.getObjects, sim.handle_all)
    if ok then
        for _, h in ipairs(objs) do
            if sim.getObjectParent(h) == -1 then
                local name = '?'
                pcall(function() name = sim.getObjectAlias(h, 0) or sim.getObjectAlias(h, 1) or '?' end)
                local t = '?'
                pcall(function()
                    local tt = sim.getObjectType(h)
                    if tt == sim.object_shape_type then t = 'shape'
                    elseif tt == sim.object_joint_type then t = 'joint'
                    elseif tt == sim.object_dummy_type then t = 'dummy'
                    else t = 'type'..tt end
                end)
                print(string.format('  /%-40s [%s] handle=%s', name, t, h))
            end
        end
    end

    -- 2. FiveCR5A_Cell 子对象
    print('')
    print('--- 2. /FiveCR5A_Cell CHILDREN ---')
    local cell = sim.getObject('/FiveCR5A_Cell')
    if cell ~= -1 then
        local ok2, children = pcall(sim.getObjectsInTree, cell, sim.handle_all, 1)
        if ok2 and children then
            for _, h in ipairs(children) do
                local name = '?'
                pcall(function() name = sim.getObjectAlias(h, 0) or sim.getObjectAlias(h, 1) or '?' end)
                local parent = sim.getObjectParent(h)
                local pname = '?'
                if parent ~= -1 then pcall(function() pname = sim.getObjectAlias(parent, 0) or '?' end) end
                print(string.format('  %-50s parent=%-25s handle=%s', name, pname, h))
            end
        end
    else
        print('  NOT FOUND!')
    end

    -- 3. 脚本
    print('')
    print('--- 3. SCRIPTS ---')
    local scriptTypes = {
        {sim.scripttype_mainscript, 'main'},
        {sim.scripttype_childscript, 'child'},
        {sim.scripttype_customizationscript, 'customization'},
    }
    for _, st in ipairs(scriptTypes) do
        local s = sim.getScript(st[1], cell ~= -1 and cell or nil)
        if s ~= -1 then
            local txt = '?'
            pcall(function() txt = sim.getScriptText(s) end)
            local firstLine = txt:match('[^\n]+') or '?'
            print(string.format('  [%s] %s', st[2], firstLine:sub(1,120)))
        end
    end
    -- 检查所有dummy上的child script
    if cell ~= -1 then
        local ok3, all = pcall(sim.getObjectsInTree, cell, sim.handle_all, 0)
        if ok3 and all then
            for _, h in ipairs(all) do
                local s = sim.getScript(sim.scripttype_childscript, h)
                if s ~= -1 then
                    local name = '?'
                    pcall(function() name = sim.getObjectAlias(h, 0) or '?' end)
                    local txt = '?'
                    pcall(function() txt = sim.getScriptText(s) end)
                    local firstLine = txt:match('[^\n]+') or '?'
                    print(string.format('  [child on %s] %s', name, firstLine:sub(1,100)))
                end
            end
        end
    end

    -- 4. Cart 对象
    print('')
    print('--- 4. CART OBJECTS ---')
    for _, name in ipairs({'CartA','CartB','CartA_SupplyPose','CartB_SupplyPose','CartA_WaitPose','CartB_WaitPose'}) do
        local h = sim.getObject('/' .. name)
        if h ~= -1 then
            local pos = {0,0,0}
            pcall(function() pos = sim.getObjectPosition(h, -1) end)
            print(string.format('  ✅ %-25s pos=(%.3f, %.3f, %.3f)', name, pos[1], pos[2], pos[3]))
        else
            print(string.format('  ❌ %-25s NOT FOUND', name))
        end
    end

    -- 5. PartsB
    print('')
    print('--- 5. PartsB (B PRODUCTS) ---')
    local partsB = sim.getObject('/FiveCR5A_Cell/PartsB')
    if partsB ~= -1 then
        local ok4, bobs = pcall(sim.getObjectsInTree, partsB, sim.handle_all, 1)
        if ok4 and bobs then
            for _, h in ipairs(bobs) do
                local name = '?'
                pcall(function() name = sim.getObjectAlias(h, 0) or sim.getObjectAlias(h, 1) or '?' end)
                print(string.format('  ✅ %s', name))
            end
        end
    else
        print('  ❌ /FiveCR5A_Cell/PartsB NOT FOUND')
    end

    -- 6. 传送带位置
    print('')
    print('--- 6. CONVEYOR POSITIONS ---')
    for _, name in ipairs({'Good_Conveyor','Defect_Conveyor'}) do
        local h = sim.getObject('/FiveCR5A_Cell/Conveyors/' .. name)
        if h == -1 then h = sim.getObject('/' .. name) end
        if h ~= -1 then
            local pos = {0,0,0}
            pcall(function() pos = sim.getObjectPosition(h, -1) end)
            print(string.format('  %-25s pos=(%.3f, %.3f, %.3f)', name, pos[1], pos[2], pos[3]))
        else
            print(string.format('  %-25s NOT FOUND', name))
        end
    end

    -- 7. R5_GOOD_PLACE
    print('')
    print('--- 7. R5 TARGET POINTS ---')
    for _, name in ipairs({'R5_GOOD_PLACE_TCP','R5_GOOD_PLACE_APP','R5_DEFECT_PLACE_TCP','R5_DEFECT_PLACE_APP'}) do
        local h = sim.getObject('/' .. name)
        if h == -1 then
            -- 搜索所有dummy
            local ok5, all = pcall(sim.getObjects, sim.handle_all)
            if ok5 and all then
                for _, oh in ipairs(all) do
                    local n = '?'
                    pcall(function() n = sim.getObjectAlias(oh, 0) or '' end)
                    if n == name then h = oh; break end
                end
            end
        end
        if h ~= -1 then
            local pos = {0,0,0}
            pcall(function() pos = sim.getObjectPosition(h, -1) end)
            print(string.format('  ✅ %-30s pos=(%.3f, %.3f, %.3f)', name, pos[1], pos[2], pos[3]))
        else
            print(string.format('  ❌ %-30s NOT FOUND', name))
        end
    end

    -- 8. String Signals
    print('')
    print('--- 8. STRING SIGNALS ---')
    for _, sig in ipairs({'cell_product_state','cell_conveyor_state','tool_cmd','joint_cmd','cart_order','product_type'}) do
        local v = sim.getStringSignal(sig)
        if v then
            print(string.format('  %-25s = "%s"', sig, v))
        else
            print(string.format('  %-25s = (nil/not set)', sig))
        end
    end

    print('')
    print('==================== DIAGNOSTIC COMPLETE ====================')
end
