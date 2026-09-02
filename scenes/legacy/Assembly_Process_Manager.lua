------------------------------------------------
-- Assembly_Process_Manager.lua
--
-- 功能：
-- 1. 控制装配过程显示
-- 2. 供料区物料不受影响
-- 3. 根据assembly_state逐步显示装配模块
--
-- 状态：
-- A_SHELL_DONE
-- A_PCB_DONE
-- A_MODULE_DONE
-- A_TERMINAL_DONE
-- A_COMPLETE
--
-- B同理
------------------------------------------------

sim=require('sim')
simROS2=require('simROS2')

local sub=nil


------------------------------------------------
-- A装配模块
------------------------------------------------

local A_Shell =
sim.getObject('/Parts/Assembly_ControlBox_Product/Assembly_ControlBox_Product_Shell')

local A_PCB =
sim.getObject('/Parts/Assembly_ControlBox_Product/Assembly_ControlBox_Product_PCB')

local A_Module =
sim.getObject('/Parts/Assembly_ControlBox_Product/Assembly_ControlBox_Product_Control_Module')

local A_Terminal =
sim.getObject('/Parts/Assembly_ControlBox_Product/Assembly_ControlBox_Product_Terminal_Block')



------------------------------------------------
-- B装配模块
------------------------------------------------

local B_Shell =
sim.getObject('/PartsB/Assembly_ControlBox_Product_B/Assembly_ControlBox_Product_Shell')

local B_PCB =
sim.getObject('/PartsB/Assembly_ControlBox_Product_B/Assembly_ControlBox_Product_PCB')

local B_Module =
sim.getObject('/PartsB/Assembly_ControlBox_Product_B/Assembly_ControlBox_Product_Control_Module')

local B_Terminal =
sim.getObject('/PartsB/Assembly_ControlBox_Product_B/Assembly_ControlBox_Product_Terminal_Block')


------------------------------------------------
-- 当前型号
------------------------------------------------

local product="A"



------------------------------------------------
-- 工具函数
------------------------------------------------

local function setVisible(obj,state)

    if obj==-1 then
        return
    end

    local list=sim.getObjectsInTree(
        obj,
        sim.handle_all,
        0
    )

    for i=1,#list do

        sim.setObjectInt32Param(
            list[i],
            sim.objintparam_visibility_layer,
            state and 1 or 0
        )

    end
end



------------------------------------------------
-- 隐藏所有装配模块
------------------------------------------------

local function hideAssembly()

    setVisible(A_Shell,false)
    setVisible(A_PCB,false)
    setVisible(A_Module,false)
    setVisible(A_Terminal,false)

    setVisible(B_Shell,false)
    setVisible(B_PCB,false)
    setVisible(B_Module,false)
    setVisible(B_Terminal,false)

end



------------------------------------------------
-- A流程
------------------------------------------------

local function A_Shell_Done()

    hideAssembly()
    setVisible(A_Shell,true)

end


local function A_PCB_Done()

    hideAssembly()

    setVisible(A_Shell,true)
    setVisible(A_PCB,true)

end


local function A_Module_Done()

    hideAssembly()

    setVisible(A_Shell,true)
    setVisible(A_PCB,true)
    setVisible(A_Module,true)

end


local function A_Terminal_Done()

    hideAssembly()

    setVisible(A_Shell,true)
    setVisible(A_PCB,true)
    setVisible(A_Module,true)
    setVisible(A_Terminal,true)

end


local function A_Complete()

    hideAssembly()

    setVisible(A_Shell,true)
    setVisible(A_PCB,true)
    setVisible(A_Module,true)
    setVisible(A_Terminal,true)

end



------------------------------------------------
-- B流程
------------------------------------------------

local function B_Shell_Done()

    hideAssembly()
    setVisible(B_Shell,true)

end


local function B_PCB_Done()

    hideAssembly()

    setVisible(B_Shell,true)
    setVisible(B_PCB,true)

end


local function B_Module_Done()

    hideAssembly()

    setVisible(B_Shell,true)
    setVisible(B_PCB,true)
    setVisible(B_Module,true)

end


local function B_Terminal_Done()

    hideAssembly()

    setVisible(B_Shell,true)
    setVisible(B_PCB,true)
    setVisible(B_Module,true)
    setVisible(B_Terminal,true)

end


local function B_Complete()

    hideAssembly()

    setVisible(B_Shell,true)
    setVisible(B_PCB,true)
    setVisible(B_Module,true)
    setVisible(B_Terminal,true)

end



------------------------------------------------
-- ROS2回调
------------------------------------------------

function assembly_callback(msg)

    local state=string.upper(msg.data)

    print("Assembly state:")
    print(state)


    if state=="A_SHELL_DONE" then
        A_Shell_Done()

    elseif state=="A_PCB_DONE" then
        A_PCB_Done()

    elseif state=="A_MODULE_DONE" then
        A_Module_Done()

    elseif state=="A_TERMINAL_DONE" then
        A_Terminal_Done()

    elseif state=="A_COMPLETE" then
        A_Complete()


    elseif state=="B_SHELL_DONE" then
        B_Shell_Done()

    elseif state=="B_PCB_DONE" then
        B_PCB_Done()

    elseif state=="B_MODULE_DONE" then
        B_Module_Done()

    elseif state=="B_TERMINAL_DONE" then
        B_Terminal_Done()

    elseif state=="B_COMPLETE" then
        B_Complete()

    end

end



------------------------------------------------
-- 初始化
------------------------------------------------

function sysCall_init()

    sub=simROS2.createSubscription(
        '/assembly_state',
        'std_msgs/msg/String',
        'assembly_callback'
    )


    print("Assembly Process Manager ready")

end



function sysCall_cleanup()

    if sub then
        simROS2.shutdownSubscription(sub)
    end

end
