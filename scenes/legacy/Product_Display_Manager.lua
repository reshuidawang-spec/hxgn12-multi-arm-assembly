-- Product_Display_Manager.lua
-- 只负责A/B供料物料显示
-- 不控制装配体

sim=require('sim')

local A={
sim.getObject('/Parts/Box_Blank'),
sim.getObject('/Parts/PCB_Supply'),
sim.getObject('/Parts/Control_Module_Supply'),
sim.getObject('/Parts/Terminal_Block_Supply')
}

local B={
sim.getObject('/PartsB/Box_Blank_B'),
sim.getObject('/PartsB/PCB_Supply_B'),
sim.getObject('/PartsB/Control_Module_Supply_B'),
sim.getObject('/PartsB/Terminal_Block_Supply_B')
}


local Assembly_A=
sim.getObject('/Parts/Assembly_ControlBox_Product')

local Assembly_B=
sim.getObject('/PartsB/Assembly_ControlBox_Product_B')


local function visible(obj,s)

    if obj==-1 then return end

    local list=sim.getObjectsInTree(obj,sim.handle_all,0)

    for i=1,#list do
        sim.setObjectInt32Param(
            list[i],
            sim.objintparam_visibility_layer,
            s and 1 or 0
        )
    end
end


local function groupVisible(g,s)

    for i=1,#g do
        visible(g[i],s)
    end
end


function hideAll()

    groupVisible(A,false)
    groupVisible(B,false)
    visible(Assembly_A,false)
    visible(Assembly_B,false)

end


function showA()

    hideAll()
    groupVisible(A,true)

end


function showB()

    hideAll()
    groupVisible(B,true)

end


function sysCall_init()

    hideAll()
    showA()

end


function sysCall_actuation()

    local p=sim.getStringSignal('product_type')

    if p then

        if p=="A" then
            showA()
        elseif p=="B" then
            showB()
        end

        sim.clearStringSignal('product_type')

    end

end
