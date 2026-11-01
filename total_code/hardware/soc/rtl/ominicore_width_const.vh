

// 这些本应是 localparam，但偶尔需要在模块之间传递标志位，因此改以
// parameter 形式提供。虽然不优雅，但作用域卫生仍好过预处理器。这些参数
// 不应改动其默认值。

parameter W_REGADDR = 5,

parameter W_ALUOP   = 6,
parameter W_ALUSRC  = 1,
parameter W_MEMOP   = 5,
parameter W_BCOND   = 2,
parameter W_SHAMT   = 5,

parameter W_EXCEPT  = 4,
parameter W_MULOP   = 3
