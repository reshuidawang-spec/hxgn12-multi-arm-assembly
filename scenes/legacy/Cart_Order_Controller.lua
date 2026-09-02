-- Cart_Order_Controller.lua
-- 信号驱动的小车运动控制器，遵循 Product_Stage_Controller 模式
-- 使用：挂载为 Non-threaded child script，仿真时一直启用
-- 读取 'cart_order' String Signal，移动 CartA/CartB

sim = require('sim')

local SPEED = 0.3          -- m/s
local TOLERANCE = 0.005    -- 5mm

-- 小车和目标的句柄（延迟获取）
local CartA, CartB
local A_Supply, A_Wait, B_Supply, B_Wait

-- 当前运动任务
local task = {cart = -1, target = -1, moving = false}

------------------------------------------------
-- 工具函数
------------------------------------------------
local function safeGet(path)
    local ok, h = pcall(sim.getObject, path)
    if ok and h ~= -1 then return h end
    return -1
end

local function moveCart(cart, target)
    if cart == -1 or target == -1 then
        print('[CART CTRL] WARNING: invalid cart or target handle')
        return
    end
    task.cart = cart
    task.target = target
    task.moving = true
    print(string.format('[CART CTRL] moving cart to target (dist=%.3f)',
        math.sqrt(
            (sim.getObjectPosition(target,-1)[1] - sim.getObjectPosition(cart,-1)[1])^2 +
            (sim.getObjectPosition(target,-1)[2] - sim.getObjectPosition(cart,-1)[2])^2
        )))
end

------------------------------------------------
-- 处理 cart_order 信号
------------------------------------------------
local function handleCartOrder(order)
    if not order then return end

    if order == 'cart_a_supply' then
        -- CartA 去供料位，CartB 去等待位
        moveCart(CartA, A_Supply)
        moveCart(CartB, B_Wait)
        print('[CART CTRL] Order: CartA → supply, CartB → wait')

    elseif order == 'cart_b_supply' then
        -- CartB 去供料位，CartA 去等待位
        moveCart(CartB, B_Supply)
        moveCart(CartA, A_Wait)
        print('[CART CTRL] Order: CartB → supply, CartA → wait')

    elseif order == 'cart_reset' then
        -- 两车都回等待位
        moveCart(CartA, A_Wait)
        moveCart(CartB, B_Wait)
        print('[CART CTRL] Order: both carts → wait')
    end
end

------------------------------------------------
-- 初始化
------------------------------------------------
function sysCall_init()
    print('===== Cart Order Controller =====')

    -- 获取手柄
    CartA    = safeGet('/CartA')
    CartB    = safeGet('/CartB')
    A_Supply = safeGet('/CartA_SupplyPose')
    A_Wait   = safeGet('/CartA_WaitPose')
    B_Supply = safeGet('/CartB_SupplyPose')
    B_Wait   = safeGet('/CartB_WaitPose')

    -- 检查
    local missing = {}
    if CartA == -1 then missing[#missing+1] = '/CartA' end
    if CartB == -1 then missing[#missing+1] = '/CartB' end
    if A_Supply == -1 then missing[#missing+1] = '/CartA_SupplyPose' end
    if A_Wait == -1 then missing[#missing+1] = '/CartA_WaitPose' end
    if B_Supply == -1 then missing[#missing+1] = '/CartB_SupplyPose' end
    if B_Wait == -1 then missing[#missing+1] = '/CartB_WaitPose' end

    if #missing > 0 then
        print('[CART CTRL] WARNING: missing objects:')
        for _, m in ipairs(missing) do
            print('  - ' .. m)
        end
        print('[CART CTRL] Import carts and run Cart_Targets_Setup first.')
    else
        print('[CART CTRL] All objects found. Ready.')
    end
end

------------------------------------------------
-- 每帧更新
------------------------------------------------
function sysCall_actuation()
    -- 读取信号
    local order = sim.getStringSignal('cart_order')
    if order then
        handleCartOrder(order)
        sim.clearStringSignal('cart_order')
    end

    -- 执行运动
    if not task.moving then return end
    if task.cart == -1 or task.target == -1 then
        task.moving = false
        return
    end

    local p = sim.getObjectPosition(task.cart, -1)
    local g = sim.getObjectPosition(task.target, -1)

    local dx = g[1] - p[1]
    local dy = g[2] - p[2]
    local dz = g[3] - p[3]
    local d = math.sqrt(dx * dx + dy * dy + dz * dz)

    if d < TOLERANCE then
        sim.setObjectPosition(task.cart, -1, g)
        task.moving = false
        print('[CART CTRL] arrived')
        return
    end

    local step = SPEED * sim.getSimulationTimeStep()
    if step > d then step = d end

    sim.setObjectPosition(task.cart, -1, {
        p[1] + dx / d * step,
        p[2] + dy / d * step,
        p[3] + dz / d * step
    })
end

function sysCall_cleanup()
end
