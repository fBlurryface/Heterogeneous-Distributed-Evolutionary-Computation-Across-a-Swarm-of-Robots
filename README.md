# Heterogeneous-Distributed-Evolutionary-Computation-Across-a-Swarm-of-Robots
This is my MSc Robotics project repository
flowchart LR
  %% High-level context
  subgraph Context
    op["Operator"]
    term["Serial Terminal"]
    fw["Firmware Runtime (Arduino)"]
    ir["IR Module"]
    peers["Neighbor Nodes"]
  end

  op -->|Uses| term
  term -->|Serial link| fw
  fw -->|I2C read/write| ir
  ir -->|IR broadcast| peers
  peers -->|IR response| ir

  %% Runtime internals (function-level sketch)
  subgraph "Framework · Runtime"
    setup[setup()]
    loop[loop()]
    quickSeedADC[quickSeedADC()]
    i2cWrite1[i2cWrite1()]
    serialSourcePoll[serialSourcePoll()]
    dispatchCommand[dispatchCommand()]
    task_rx_poll[task_rx_poll()]
  end

  setup -->|seed RNG| quickSeedADC
  setup -->|"MODE_START_RX / CLEAR_MSG*"| i2cWrite1
  loop -->|poll serial| serialSourcePoll
  loop -->|dispatch| dispatchCommand
  loop -->|poll incoming segments| task_rx_poll
