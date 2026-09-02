-- Assembly_Display_Manager.lua
-- 后续R3完成后显示总装配体
-- 当前只预留接口

sim=require('sim')

local Assembly_A=sim.getObject('/Parts/Assembly_ControlBox_Product')
local Assembly_B=sim.getObject('/PartsB/Assembly_ControlBox_Product_B')


local function visible(obj,s)

    local list=sim.getObjectsInTree(obj,sim.handle_all,0)

    for i=1,#list do
        sim.setObjectInt32Param(
            list[i],
            sim.objintparam_visibility_layer,
            s and 1 or 0
        )
    end
end


function sysCall_actuation()

    local state=sim.getStringSignal('assembly_state')

    if state then

        if state=="A_COMPLETE" then
            visible(Assembly_A,true)
        end

        if state=="B_COMPLETE" then
            visible(Assembly_B,true)
        end

        sim.clearStringSignal('assembly_state')
    end

end
