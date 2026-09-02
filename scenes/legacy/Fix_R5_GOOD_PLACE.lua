-- Fix_R5_GOOD_PLACE.lua
-- 将 R5_GOOD_PLACE_TCP 和 _APP 的 X 坐标从 0.85 更新到 0.35
-- （同步 Good_Conveyor 从 X=0.98 到 X=0.48 的 -0.5m 偏移）
-- 运行一次后禁用

sim = require('sim')

function sysCall_init()
    print('===== Fix R5 Good Place Position =====')

    local targets = {
        {name = 'R5_GOOD_PLACE_TCP', oldX = 0.85, newX = 0.35},
        {name = 'R5_GOOD_PLACE_APP', oldX = 0.85, newX = 0.35},
    }

    for _, t in ipairs(targets) do
        local h = sim.getObject('/' .. t.name)
        if h == -1 then
            -- 搜索场景中所有对象
            local objs = sim.getObjects(sim.handle_all)
            for _, oh in ipairs(objs) do
                local n = '?'
                pcall(function() n = sim.getObjectAlias(oh, 0) or '' end)
                if n == t.name then h = oh; break end
            end
        end

        if h ~= -1 then
            local oldPos = sim.getObjectPosition(h, -1)
            local newPos = {t.newX, oldPos[2], oldPos[3]}
            sim.setObjectPosition(h, -1, newPos)
            print(string.format('✅ %s: (%.3f,%.3f,%.3f) → (%.3f,%.3f,%.3f)',
                t.name, oldPos[1], oldPos[2], oldPos[3],
                newPos[1], newPos[2], newPos[3]))
        else
            print('❌ ' .. t.name .. ' not found (may already be fixed)')
        end
    end

    print('===== Fix complete. Save scene (Ctrl+S) and disable this script. =====')
end
