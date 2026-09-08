function sysCall_info()
    return {
        autoStart = true,
        menu = 'CR5 assembly\nAudit loaded scene',
    }
end

local function writeAudit()
    if auditDone then return end
    local scenePath = sim.getStringParam(sim.stringparam_scene_path_and_name)
    if scenePath == nil or scenePath == '' then return end
    auditDone = true
    local counts = {}
    local objects = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
    for _, handle in ipairs(objects) do
        local alias = sim.getObjectAlias(handle, 0)
        counts[alias] = (counts[alias] or 0) + 1
    end
    local names = {
        'R1', 'R2', 'R3', 'R4', 'R5', 'R6', 'R7', 'R8',
        'Central_Indexing_Conveyor', 'Indexing_Pallet_1',
        'Public_Workspace_1', 'Public_Workspace_2', 'Public_Workspace_3',
        'R8_vacuum_tip', 'R8T_vacuum_cup',
        'RobotBases', 'R1_Base', 'R2_Base', 'R3_Base', 'R4_Base',
        'R5_Base', 'R6_Base', 'R7_Base', 'R8_Base',
        'WB1_Table', 'Damping_Table_Left', 'Damping_Table_Right',
    }
    local output = assert(io.open('/tmp/cr5_loaded_scene_audit.tsv', 'w'))
    output:write('scene\t' .. scenePath .. '\n')
    output:write('objects\t' .. tostring(#objects) .. '\n')
    for _, name in ipairs(names) do
        output:write(name .. '\t' .. tostring(counts[name] or 0) .. '\n')
    end
    output:close()
    sim.addLog(sim.verbosity_scriptinfos, 'CR5 loaded-scene audit completed')
end

function sysCall_init()
    sim = require('sim')
    auditDone = false
    writeAudit()
end

function sysCall_afterInstanceSwitch()
    writeAudit()
end

function sysCall_nonSimulation()
    writeAudit()
    if auditDone then return {cmd = 'cleanup'} end
end
