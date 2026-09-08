function sysCall_info()
    return {
        autoStart = true,
        menu = 'CR5 assembly\nClean loaded scene helpers',
    }
end

function sysCall_init()
    sim = require('sim')
    cleanupDone = false
end

local function cleanLoadedScene()
    if cleanupDone then return {cmd = 'cleanup'} end
    local scenePath = sim.getStringParam(sim.stringparam_scene_path_and_name)
    if scenePath ~= '/home/zhu/cr5_assembly_team/scenes/compact_cell.ttt' then
        return
    end
    cleanupDone = true
    local transient = {
        Motion_Collision_Planner = true,
        Assembly_Collision_Planner = true,
        Assembly_Runtime_Batch = true,
    }
    local disabled = {}
    local objects = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
    -- Never save an initialization-stage empty scene.  The target production
    -- scene currently has hundreds of objects and all eight robot roots.
    if #objects < 500 then return end
    for index = 1, 8 do
        local found = false
        for _, handle in ipairs(objects) do
            if sim.getObjectAlias(handle, 0) == 'R' .. tostring(index) then
                found = true
                break
            end
        end
        if not found then return end
    end
    for _, handle in ipairs(objects) do
        local alias = sim.getObjectAlias(handle, 0)
        if transient[alias] then table.insert(disabled, handle) end
    end
    for _, handle in ipairs(disabled) do
        sim.setObjectInt32Param(handle, sim.scriptintparam_enabled, 0)
    end
    sim.saveScene(scenePath)
    sim.addLog(
        sim.verbosity_scriptinfos,
        string.format('CR5 loaded scene saved; disabled %d helper(s)', #disabled)
    )
    return {cmd = 'cleanup'}
end

function sysCall_afterInstanceSwitch()
    return cleanLoadedScene()
end

function sysCall_nonSimulation()
    return cleanLoadedScene()
end
