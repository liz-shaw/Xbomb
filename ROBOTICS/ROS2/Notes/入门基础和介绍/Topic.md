#  topic

## 命令查看主题

- `ros2 topic list`

  - 示例

    - 不运行任何节点时, 输出

      - ```bash
        /parameter_events
        /rosout
        ```

    - 运行节点

      - ```bash
        ros2 run my_py_pkg robot_news_station 
        ```

      - ```bash
        #输出
        arameter_events
        /robot_news
        /rosout
        ```

    - 终止节点后，输出

      - ```bash
        /parameter_events
        /rosout
        ```

  - 至少有一个订阅者或者发布者节点运行的时候，才会在节点里看到这个主题

- `ros2 topic info /topic_name`

  - 输出

    - ```bash
      username:~$ ros2 topic info /robot_news
      Type: example_interfaces/msg/String
      Publisher count: 1
      Subscription count: 0
      
      ```

    - 消息类型

    - 发布者数量

    - 订阅者数量

- `ros2 topic echo /topic_name`
  - 收到来自发布者的信息
  - 继续运行`ros2 topic info /topic_name`
    - 订阅者数量增加

- `ros2 interface show example_interfaces/msg/String`

  - 可以查看接口的详细信息

  - ```bash
    username:~$ ros2 interface show example_interfaces/msg/String
    # This is an example message of using a primitive datatype, string.
    # If you want to test with this that's fine, but if you are deploying
    # it into a system you should create a semantically meaningful message type.
    # If you want to embed it in another message, use the primitive data type instead.
    string data  #a string datatype with a name:data
    
    ```

- `ros2 topic hz /topic_name`

  - 查看主题发送的频率： Hertz

  - ```bash
    #取一行结果
    average rate: 2.000
    	min: 0.499s max: 0.501s std dev: 0.00034s window: 136
    
    ```

  - average rate:单位是赫兹

- `ros2 topic bw /topic_name`

  - bandwidth

- `ros2 topic pub -r 5 /robot_news example_interfaces/msg/String "data: 'hello from the terminal'"`

  - `pub`: publisher
  - `-r` :rate
  - `5`: 5 hz
  - 从终端



## remap at runtime

### rename a node

- `ros2 run my_py_pkg robot_news_station --ros-args -r __node:=my_station`

- `--ros-args`: 表明后面的参数不是自定义
- `-r`:  remap
  - 也可以写成`--remap`
- `__node:=my_station`
  - `__node`: 变量:节点自己 的名字
  - `:=`:重映射赋值
- 作用:启动节点并且修改其节点名称
  - 注意:这种修改不是永久的,下一次用`ros2 run my_py_pkg robot_news_station`, 节点名称仍然是`robot_news_station`

### rename/remap a topic of a publisher

`ros2 run my_py_pkg robot_news_station --ros-args -r __node:=my_station -r robot_news:=abc`

- 主题名字修改
- 启动原来`robot_news`主题对应的订阅者
  - `ros2 run my_py_pkg smartphone`
  - 结果: 啥也没收到
- 查看topic list:
  - `ros2 topic list`
  - 结果: 多了两个主题`robot_news`, `abc`



### rename/remap a topic of a subscriber

`ros2 run my_py_pkg smartphone --ros-args -r robot_news:=abc`

- 就可以正常接收publisher的信息了



## rqt/rqt_graph

### 显示选项

- group: 如果你有命名空间（Namespace），它会把属于同一个组的节点框在一起

- hide

  - Debug：隐藏掉系统自带的调试信息（通常建议勾选
  - Leaf topics: 只有发布, 没有被订阅的话题
  - Unreachable: 隐藏和主网络断开连接的部分
  - Dead sinks:  自己运行、不跟别人发消息也不收消息的“孤岛”节点

  





