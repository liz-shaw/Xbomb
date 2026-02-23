# CMakeList.txt介绍

> 笔记中的主要内容源于：[Blog by Matt Morse](https://mjmorse.com/blog/cmake-template/)

## 角色

- 是“八面玲珑的”配置文件，屏蔽了底层系统（比如Linux， Windows）和构建工具的差异（比如Make, Ninja)
  - Cmake读取CMakeList.txt后，会根据当前环境生成对应的构建脚本（如 `Makefile` 或 `.sln` 项目文件）

## 功能

### 功能介绍

- **Compile** source code into a static library
  将源代码编译成静态库
- **Link** the source code into an executable
  将源代码链接到可执行文件中
- Handle unit **testing** 处理单元测试
- **Import** third party libraries and their **dependencies** transitively
  传递导入第三方库*及其依赖项*
- **Export** the static library and its dependencies transitively
  以传递方式导出静态库*及其依赖项*

### 传递性（transitive

- **举个例子**：如果我在项目中导入了 `LibA` ，而 `LibB` 是 `LibA` 的依赖项，那么 `LibB` 也会自动包含在我的项目中

-  传递的内容是什么

  - **头文件搜索路径** (`INTERFACE_INCLUDE_DIRECTORIES`
  - **编译预处理宏** (`INTERFACE_COMPILE_DEFINITIONS`)
  - **编译选项**（如 C++ 标准、优化等级）
  - **二进制库链接** (`INTERFACE_LINK_LIBRARIES`)

  

## 核心架构：基于“目标（Target）”

- **Target (目标)**：可以是可执行文件 (`add_executable`) 或库 (`add_library`)。

- **Property (属性)**：每个 Target 都有自己的属性，如编译选项、宏定义、包含路径。

- **Transitive Usage Requirements (传递性需求)**：

  - `PRIVATE`：仅在编译当前目标时使用。

  - `INTERFACE`：自己编译不用，但链接我的目标需要用。

  - `PUBLIC`：两者兼之。

> 作用分析：这种机制确保了依赖关系的纯净性，避免了大型项目中的头文件污染



# CMake的一些语法和conventions



## 工程conventions

### 目标导向（Target-centric, not file-centric）

- 文件导向

  ```cmake
  include_directories(...)
  link_directories(...)
  add_definitions(...)
  ```

- 目标导向

    ```cmake
    target_include_directories()
    target_link_libraries()
    target_compile_definitions()
    target_compile_options()
    ```

- 其他注意事项
  - Target 是一切的中心
  - 依赖dependencies是图结构（DAG有向无环），不是线性顺序
  - 属性property是附着在 target 上的，不是全局变量
    - 编译选项、包含路径、链接库不是全局变量，而是属于 Target 的属性



### 接口思维（interface

- 每个 target 都有三类属性：

```cmake
PRIVATE    = 只影响自己,实现文件
PUBLIC     = 影响自己 + 依赖者,api
INTERFACE  = 只影响依赖者，头文件
```

- | **关键字**    | **对内 (构建自己时)** | **对外 (别人链接我时)** | **典型场景**                           |
  | ------------- | --------------------- | ----------------------- | -------------------------------------- |
  | **PRIVATE**   | ✅ 有效                | ❌ 无效                  | 仅在 `.cpp` 中使用的头文件/库          |
  | **INTERFACE** | ❌ 无效                | ✅ 有效                  | 只有 `.h` 的模板库 (Header-only)       |
  | **PUBLIC**    | ✅ 有效                | ✅ 有效                  | 你的头文件里 `#include` 了别人的头文件 |

- 例子：

```cmake
target_include_directories(lib
    PUBLIC include
    PRIVATE src
)
```



###  不污染全局（No Global State）

- 全局状态 = 工程复杂度指数级爆炸源头

- **Property 是属性：** 编译选项、包含路径、链接库不再是全局变量，而是属于 Target 的属性
- **不要使用全局指令：** 
  - 避免：`include_directories()`, `link_libraries()`, `add_definitions()`。
  - 推荐：`target_include_directories()`, `target_link_libraries()`, `target_compile_options()`

### 目录conventions

```bash
project/
├── CMakeLists.txt        # 顶层调度
├── cmake/                # 工具函数 & 宏
├── src/                  # 实现
├── include/              # 头文件（对外接口）
├── third_party/          # 第三方依赖
├── test/                 # 测试
└── build/                # 构建目录（out-of-source）
```

- 源码src/ 和build/, test/, include/,install/ ,log/等等分开

  - 永远不要在src下面build( in-sourse build × )

- build/ 不进 git

  - 将其加入 `.gitignore`文件

- 模块化：  

  - 子模块管理：add subdirectory

    - 每个子目录都应该有一个独立的 `CMakeLists.txt`

    - 细节模块不可存在于根目录（层级结构有利于管理

    - 例子

      - ```cmake
        add_subdirectory(core)
        add_subdirectory(net)
        add_subdirectory(app)
        ```

      - 依赖方向

        - core → net → app

  -  每个模块自管理源码

    - 如果变动文件
      - 只改自己目录的 `CMakeLists.txt`
      - 不动根目录
      - 不影响别的模块



### 依赖项管理conventions

- **优先使用 `find_package()`：** 这是寻**找已安装库**的标准方式（基于 Config 或 Find 模式）。

- **FetchContent (推荐)：** 对于小型或需要自动化下载的依赖，使用 CMake 原生的 `FetchContent` 模块

- **不要硬编码路径：** 绝对不要写 `include_directories(/usr/local/include)`。

- **提供 Config 文件：** 如果你**开发的是库**，确保生成 `MyLibConfig.cmake`，**让别人能通过 `find_package(MyLib)` 找到你**

  

### 版本要求与声明 (Boilerplate)

- 每个根目录 `CMakeLists.txt` 的标准开头

  - ```cmake
    cmake_minimum_required(VERSION 3.15...3.25) # 指定版本范围
    
    project(MyProject 
        VERSION 1.0.0 
        LANGUAGES CXX
    )
    
    # 设置 C++ 标准（不要直接修改 CMAKE_CXX_FLAGS）
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    ```

  - 



## 变量与语法规范

### 作用域

#### 目录级作用域

- 每一个 `add_subdirectory()` 都会创建一个新的“子作用域”

-  子目录会**拷贝**一份父目录当前所有变量的副本

- **隔离性：** 子目录对变量的修改，默认只在子目录及其孙子目录有效。回到父目录时，变量依然是原来的样子

  - 提升到父级作用域

    - ```cmake
      set(MY_VAR "NEW_VALUE" PARENT_SCOPE)
      ```

| **作用域类型**        | **开启方式**       | **变量可见范围**     | **销毁时机**        |
| --------------------- | ------------------ | -------------------- | ------------------- |
| **目录级 (Local)**    | `add_subdirectory` | 当前目录及后续子目录 | 该目录解析完成      |
| **函数级 (Function)** | `function()`       | 仅函数内部           | 函数运行结束        |
| **全局级 (Cache)**    | `set(... CACHE)`   | **全工程**所有位置   | 手动删除 build 目录 |





### 变量

- CMake 里的变量本质上都是**字符串**，主要有三类变量

- **普通变量 normal**

  - 辨认： 什么标记都没带

  - `set(<变量名> <值1> <值2> ...)`

  - 存储位置： 内存

  - 生命周期： 随着当前目录的 `CMakeLists.txt` 或函数（Function）执行结束而销毁。

  - ```bash
    # 例子：声明一个源文件列表（实际上是字符串 "a.cpp;b.cpp"）
    set(MY_SRC_FILES "main.cpp" "util.cpp") 
    
    # 在子目录修改，父目录无感知
    set(MY_VAR "NewValue" PARENT_SCOPE) # 除非显式使用这个关键字
    ```

    

- **缓存变量 cache**

  - 辨认： CACHE

  - `set(<变量名> <初始值> CACHE <类型> "<描述文字>" [FORCE])`

  - 存储位置： 硬盘（构建目录下的 `CMakeCache.txt`

  - 声明周期：**持久化**。除非手动删除 `build` 目录或显式清除缓存

  - 例子

    - ```cmake
      # 例子：定义一个给用户看的开关
      set(USE_MY_LIB ON CACHE BOOL "Whether to link my custom library")
      
      # 强制更新缓存（即使之前已经存在）
      set(MY_VERSION "1.2.0" CACHE STRING "Version" FORCE)
      ```

- **环境变量 env**

  - 辨认： ENV

  - `set(ENV{<变量名>} <值>)`

  - 存储位置： 当前 Shell 进程的环境变量块

  - **生命周期：** 仅在当前 CMake 配置程序运行期间有效（配置结束后，系统环境变量不会改变）

  - 例子

    - ```cmake
      # 引用时必须带 ENV 关键字
      message(STATUS "Home dir: $ENV{HOME}")
      
      # 临时修改（仅影响后续的 CMake 查找指令）
      set(ENV{CC} "/usr/bin/clang")
      ```

    - 

- 对比

  - | **维度**     | **普通变量 (Normal)**          | **缓存变量 (Cache)**          | **环境变量 (Environment)**  |
    | ------------ | ------------------------------ | ----------------------------- | --------------------------- |
    | **引用方式** | `${VAR}`                       | `${VAR}`                      | **`$ENV{VAR}`**             |
    | **作用域**   | **局部**（向下传播）           | **全局**（跨目录/跨运行）     | **外部**（当前进程）        |
    | **用户交互** | 不可见                         | **可见**（命令行 `-D` / GUI） | 不可见（除非在 Shell 设定） |
    | **读写速度** | 最快 (内存)                    | 较慢 (硬盘读取)               | 一般                        |
    | **覆盖关系** | **同名普通变量会屏蔽缓存变量** | 只能通过 `FORCE` 覆盖普通变量 | 互不影响                    |

### 指令语句

#### 变量操作

- **设置变量**：使用 `set()` 指令。

  ```CMake
  set(MY_VAR "hello")         # 单值
  set(MY_LIST "a" "b" "c")    # 列表（实质是以分号分隔的字符串 "a;b;c"）
  ```

- **引用变量**：使用 `${}` 符号。

  ```CMake
  message("Value: ${MY_VAR}")
  ```

#### flow control

- if语句

  - 在 `if` 语句中引用变量，通常直接写变量名，不需要加 `${}`

  - ```cmake
    if(MSVC)
        message("Running on Windows with MSVC")
    elseif(APPLE)
        message("Running on macOS")
    else()
        message("Other platform")
    endif()
    ```

    

- 循环

  - **遍历列表**：

    ```CMake
    set(SOURCES main.cpp util.cpp)
    foreach(f ${SOURCES})
        message("Source file: ${f}")
    endforeach()
    ```

    - `foreach(f ${SOURCES}) ... endforeach()`:遍历循环
      - 等价于c++中: `for(auto const& f :SOURCES )`

  - **范围循环**：

    ```CMake
    foreach(i RANGE 1 10) # 1到10
        message("Index: ${i}")
    endforeach()
    ```

#### 函数和宏定义

##### 函数 (Function)

- **特点**：拥有独立的作用域。在函数内部修改变量不会影响外部。

```CMake
function(my_function arg1)
    message("Argument: ${arg1}")
    set(INTERNAL_VAR "I am local") # 外部访问不到
endfunction()
```

- ```cmake
  function(<函数名> 参数1 参数2 ...)
  	<函数体>
  endfunction()
  ```

- `INTERNAL_VAR`: 函数栈帧私有的变量

- 形参

  - `arg1` 是你在定义函数时给**第一个参数**起的**名字**



##### 宏 (Macro)

- **特点**：类似 C 语言的宏替换。它直接将代码嵌入调用处，**没有**独立作用域。



```CMake
macro(my_macro arg1)
    set(GLOBAL_LIKE_VAR "I affect the caller")
endmacro()
```



##### 参数处理

- 在函数/宏内部，CMake 提供了特殊的预定义变量：

  - `ARGC`：参数总数。

  - `ARGV`：所有参数的列表。

  - `ARGN`：除了显式声明的参数外，多出来的参数列表。





# template文件讲解

## 文件树

```bash
.
├── cmake
│   ├── CMakeDemo-config.cmake
│   └── FindCMakeDemo.cmake
├── CMakeLists.txt
├── include
│   ├── CMakeLists.txt
│   └── source_file.hpp
├── LICENSE
├── README.md
├── src
│   ├── CMakeLists.txt
│   └── source_file.cpp
└── tests
    ├── catch.hpp
    ├── CMakeLists.txt
    └── test_cmake_demo.cpp

```



## 源文件和包含文件中的CMakeList.txt

- 二者几乎相同

  

### 文件代码

- ./src/CMakeList.txt

- ./include/CMakeList.txt

- ```cmake
  # 显式列出 `CMakeDemo_SRC` 中的所有源文件。
  # 这非常重要，因为 CMake 并不是一个“构建系统”，它是一个“构建系统生成器”。
  # 假设你在运行完 `cmake ..` 之后，又在 src/ 目录下添加了一个 foo.cpp 文件。
  # 如果你使用 `file(GLOB ... )` 这种自动搜寻方式，CMake 的变动不会传递给底层的 Makefile；
  # Makefile 根本不知道新文件 foo.cpp 的存在，因此不会自动重新运行 CMake 来更新项目。
  # 这会导致你同事在编译时报错，而且由于逻辑隐蔽，很难查出原因。
  # 
  # 无论你是否使用 `file(GLOB ...)`，最终都需要重新运行 CMake。
  # 但通过“显式列举文件列表”的方式，你可以在编译前就明确知道哪些文件参与了构建，
  # 从而避免由于自动搜寻带来的不可控错误。
  set(CMakeDemo_SRC
      source_file.cpp
      # 如果你以后加了新文件，记得写在这里，例如：
      # hello.cpp
      )
  
  # 为这些源文件补全完整路径（例如变成 /home/user/project/src/source_file.cpp）
  PREPEND(CMakeDemo_SRC)
  
  # 将定义好的源文件变量传递给“父级作用域”。
  # 这样根目录的 CMakeLists.txt 才能看到这个列表，并用它来生成库文件。
  set(CMakeDemo_SRC ${CMakeDemo_SRC} PARENT_SCOPE)
  ```



### 指令解析

- 源代码清单

  - ```cma
    set(CMakeDemo_SRC 
    	source_file.cpp)
    ```

  - set()
    - 定义一个变量，并且给他赋值
    - 创建一个名为 `CMakeDemo_SRC` 的变量，并给它赋值
  - 声明当前目录下哪些 `.cpp` 文件属于这个项目

- 路径补全

  - ```cmake
    PREPEND(CMakeDemo_SRC)
    ```

  - PREPEND(...)
    - 给文件名补全路径，这是一个自定义宏/函数
  - 为什么要补路径
    - 现在是在 `src/` 文件夹里，如果你只写 `source_file.cpp`，根目录的 CMake 在构建时会找不到文件
    - 这个会把 `source_file.cpp` 变成类似 `src/source_file.cpp` 这样的全路径，确保“总公司”能按图索骥找到文件

- 导出数据（把变量传到父作用域scope

  - ```bash
    set(... PARENT_SCOPE)
    ```

  - 把变量传递到**父作用域**

  - 不写会怎样?

    - 根目录 `CMakeLists.txt` 看不到这个变量 
    - 只能在当前子目录用 



## 测试代码中的CMakeList.txt

- `tests/CMakeLists.txt`
- 特殊之处： 包含了第一个目标定义： `TestCMakeDemo`

### 文件代码

```cmake
# 指定 CMake 的最低版本要求为 3.1
cmake_minimum_required(VERSION 3.1)

# 设置项目使用的 C++ 标准为 C++11
set(CMAKE_CXX_STANDARD 11)

# 显式列出测试用的源文件和头文件。
# 这里使用了 Catch 单元测试框架（Header-only），它与测试源码存放在同一目录下。
set(CMakeDemo_TEST_SRC
    test_cmake_demo.cpp
)

set(CMakeDemo_TEST_HEADER
    catch.hpp
)

# 调用自定义宏/函数 PREPEND，为文件名补全当前路径前缀
# 确保在根目录构建时能正确找到这些文件
PREPEND(CMakeDemo_TEST_SRC)

# 创建一个可执行目标（Target），该目标由上述指定的测试源码和头文件编译而成
add_executable(TestCMakeDemo ${CMakeDemo_TEST_SRC} ${CMakeDemo_TEST_HEADER})

# 启用 CMake 的内置测试功能（CTest 模块）
enable_testing()

# 将该可执行程序注册到 CTest 中，使其可以通过 "ctest" 命令运行
add_test(NAME TestCMakeDemo COMMAND TestCMakeDemo)

# 将测试程序链接到我们之前编译好的主业务库 CMakeDemo 上
# 这样测试程序才能调用主库中的函数和类
target_link_libraries(TestCMakeDemo CMakeDemo)
```

### 指令解析

- 编译环境约束

  - ```cmake
    cmake_minimum_required(VERSION 3.1)
    set(CMAKE_CXX_STANDARD 11)
    ```

- 源代码和路径预处理

  - ```cmake
    set(CMakeDemo_TEST_SRC test_cmake_demo.cpp)
    set(CMakeDemo_TEST_HEADER catch.hpp)
    PREPEND(CMakeDemo_TEST_SRC)
    ```

  - `PREPEND` 宏通常将 `${CMAKE_CURRENT_SOURCE_DIR}/` 注入每个文件名，确保在跨目录链接或多配置生成时，编译器能定位到真实的物理文件

- 目标建模

  - ```cmake
    add_executable(TestCMakeDemo ${CMakeDemo_TEST_SRC} ${CMakeDemo_TEST_HEADER})
    ```

  - `TestCMakeDemo`: 最终生成的exe的文件名
  - `${..._SRC}`: .cpp文件，实现
  - `${..._SRC}`: .hpp文件，接口

- 依赖项depend和property传播

  - ```cmake
    target_link_libraries(TestCMakeDemo CMakeDemo)
    ```

  - link 

    - TestCMakeDemo: 接收者
    - CMakeDemo： 提供者

- 测试框架集成

  - ```cmake
    enable_testing()
    add_test(NAME TestCMakeDemo COMMAND TestCMakeDemo)
    ```

  - `enable_testing()`: 在构建build目录里生成一个特殊的文件（叫 `CTestTestfile.cmake`）

  - add_test(...)

    - `NAME TestCMakeDemo`：给这个测试任务起的逻辑名字。你在查看**测试报告**时，会看到这个名字。
    - `COMMAND TestCMakeDemo`：这是命令内容



## 根目录下的CMakeList.txt

### 最简单的cmake项目

#### main.cpp

```cpp
#include <iostream>

int main() {
    std::cout << "Hello, CMake!" << std::endl;
    return 0;
}
```

#### CMakeList.txt

```cmake
# 1. 规定 CMake 的最低版本（防止老版本带不动新指令）
cmake_minimum_required(VERSION 3.10)

# 2. 给你的项目起个名字
project(MyProject)

# 3. 告诉 CMake：把 main.cpp 编译成一个叫 "hello_app" 的可执行文件, 生成hello_cpp.exe(windows)
add_executable(hello_app main.cpp)
```

#### 运行

- `mkdir build && cd build`
- `cmake ..`: 告诉 CMake，清单在上一层文件夹里
- `cmake --build .`
- 运行结果
  - 在build文件里，多出来一个hello_app文件（hello_cpp.exe windows



### 文件代码 

```cmake
# 首先指定 CMake 的最低版本要求非常重要。
# 这样用户在构建失败时，能清楚地知道是因为版本不兼容，
# 而不是去苦苦寻找某个导致失败的特定 CMake 函数。
cmake_minimum_required(VERSION 3.1)

# 设置项目全局的 C++ 标准。该标准将被项目中定义的所有目标（target）继承。
# 也可以针对特定目标指定 C++ 标准，方法是结合使用：
#   set_target_properties(foo PROPERTIES CXX_STANDARD 11)
#   target_compile_features(foo PUBLIC cxx_std_14)
set(CMAKE_CXX_STANDARD 11)

# 设置项目名称和版本号。
# 这方便库或工具的用户在引入本项目时指定特定版本，例如：
#   find_package(CMakeDemo 1.0 REQUIRED)
project(CMakeDemo VERSION 1.0)
set(CMakeDemo_VERSION 1.0)

# 引入 CTest 模块，在代码编译完成后可以通过 "make test" 启用单元测试。
include(CTest)

# 自定义函数：为子目录中的源文件自动添加子目录路径前缀
FUNCTION(PREPEND var )
    SET(listVar "")
    FOREACH(f ${${var}})
        LIST(APPEND listVar "${CMAKE_CURRENT_SOURCE_DIR}/${f}")
    ENDFOREACH(f)
    SET(${var} "${listVar}" PARENT_SCOPE) # 将修改后的变量推送至父级作用域
ENDFUNCTION(PREPEND)

# 设置构建产物的输出位置。
# 我们将可执行文件和静态库分别放置在构建目录之外的 bin/ 和 lib/ 目录中。
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/lib")

# 在系统中查找 LAPACK 库。这主要用于演示。
find_package(LAPACK REQUIRED)

# 引入源代码和头文件目录。
# 这会调用每个子目录下的 CMakeLists.txt。
# 它们可以定义自己的库、可执行文件等目标，但在此处我们在根目录定义所有可导出的目标。
add_subdirectory(src)
add_subdirectory(include)

# 添加测试目录。这是可选的，可以通过以下命令禁用：
#   cmake -DBUILD_TESTING=OFF ..
# 运行此处生成的单元测试，只需执行：
#   make test  (或者直接运行 ctest)
# 如果你的测试有控制台打印信息，可以通过以下命令查看标准输出：
#   ctest -V
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

# 定义名为 CMakeDemo 的静态库目标，
# 依赖项为 src/ 和 include/ 目录下的内容。
add_library(CMakeDemo STATIC ${CMakeDemo_SRC} ${CMakeDemo_INC})

# 以下变量用于微调安装路径，以支持多版本并存的安装。
set(CMakeDemo_INCLUDE_DEST "include/CMakeDemo-${CMakeDemo_VERSION}")
set(CMakeDemo_LIB_DEST "lib/CMakeDemo-${CMakeDemo_VERSION}")

# 包含目录需要使用“生成器表达式”，因为安装头文件后会改变包含路径。
# 指定 CMakeDemo 在编译时需要位于 include/ 目录下的文件。
# 通常写法是：target_include_directories(CMakeDemo PUBLIC include/)
# PUBLIC 意味着其他链接到 CMakeDemo 的库也应该包含该 include 目录。
# 
# 但这里有一个细节：如果我们正在将项目安装到 CMAKE_INSTALL_PREFIX，
# 我们不能再指定构建目录里的 include/ 路径。
# 我们希望其他项目包含安装后的目录。下面的命令处理了这种差异：
# $<BUILD_INTERFACE:...> 和 $<INSTALL_INTERFACE:...> 是根据
# “仅构建”还是“安装后”来自动切换路径的宏。
target_include_directories(CMakeDemo PUBLIC
    # 从源码构建时使用的头文件路径
    $<BUILD_INTERFACE:${CMakeDemo_SOURCE_DIR}/include> 
    $<BUILD_INTERFACE:${CMakeDemo_BINARY_DIR}/include> 

    # 安装后使用的头文件路径（隐式添加 ${CMAKE_INSTALL_PREFIX} 前缀）
    $<INSTALL_INTERFACE:include> 
    )

# 指定 CMakeDemo 需要链接 LAPACK 才能正常工作。
# 理想情况下，LAPACK 应该提供 LAPACK::LAPACK 目标以便链接。
# 但每个库的情况不同，必须查阅文档确定定义的变量名。
target_link_libraries(CMakeDemo ${LAPACK_LIBRARIES})

# 将 CMakeDemo 安装到 CMAKE_INSTALL_PREFIX（Linux 上默认为 /usr/local）。
# 若要更改安装位置，请运行：
#   cmake -DCMAKE_INSTALL_PREFIX=<自定义路径> ..

# install(...) 指定项目的安装规则。
# 可以指定文件安装位置、用户权限、构建配置等。此处我们仅执行拷贝。
# install(TARGETS ...) 指定目标的安装规则。
# 这里我们对 CMakeDemo 目标执行以下操作：
#   - 将共享库安装到 ${CMakeDemo_LIB_DEST}
#   - 将静态库安装到 ${CMakeDemo_LIB_DEST}
#   - 将包含目录关联到 ${CMakeDemo_INCLUDE_DEST}
# 我们还需要指定关联的“导出（export）”名；导出就是一个待安装目标的清单。
# 这里我们将 CMakeDemo 关联到名为 CMakeDemoTargets 的导出组。
install(
    # 要安装的目标
    TARGETS CMakeDemo 
    # 包含我们要安装目标的 CMake “导出组”名称
    EXPORT CMakeDemoTargets
    # 运行 "make install" 后，动态库、静态库以及包含目录的目标路径
    LIBRARY DESTINATION ${CMakeDemo_LIB_DEST}
    ARCHIVE DESTINATION ${CMakeDemo_LIB_DEST} 
    INCLUDES DESTINATION ${CMakeDemo_INCLUDE_DEST}
    )

# 现在我们需要安装上面定义的导出组 CMakeDemoTargets。
# 这是为了让其他项目能通过 find_package(CMakeDemo) 找到本项目。
# find_package(CMakeDemo) 会寻找 CMakeDemo-config.cmake 文件来获取目标信息。
# 这些信息就存储在 CMakeDemoTargets 中，所以我们也必须安装它。
# install(EXPORT ...) 会安装导出的信息。
# 此处将其保存为 CMakeDemoTargets.cmake，并为其中所有目标添加命名空间前缀 CMakeDemo::。
install(
    # 要保存的导出组（匹配上面定义的名称）
    EXPORT CMakeDemoTargets
    # 存储导出信息的 CMake 文件名
    FILE  CMakeDemoTargets.cmake
    # 命名空间前缀（后续导入时，我们将使用 CMakeDemo::CMakeDemo）
    NAMESPACE CMakeDemo::
    # 放置该文件的位置（此处与库放在一起）
    DESTINATION ${CMakeDemo_LIB_DEST}
    )

# install(FILES ...) 只是将文件拷贝到指定位置并设置属性。
install(FILES ${CMakeDemo_INC} DESTINATION ${CMakeDemo_INCLUDE_DEST})

# 生成“版本文件”，以防用户只想加载特定版本的 CMakeDemo。
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    CMakeDemoConfigVersion.cmake
    VERSION ${CMakeDemo_VERSION}
    COMPATIBILITY AnyNewerVersion
    )

# 将生成的 CMake 配置文件拷贝到已安装的库目录中
install(
    FILES 
        "cmake/CMakeDemo-config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/CMakeDemoConfigVersion.cmake"
    DESTINATION ${CMakeDemo_LIB_DEST}
    )
```

### 指令解析

- 版本信息和输出路径
  - `cmake_minimum_required(VERSION 3.1)`： Cmake版本
  - `set(CMAKE_CXX_STANDARD 11)`： c++版本
  - `project(CMakeDemo VERSION 1.0)`： 项目版本，用于后续安装路径和版本文件的生成
  - 
- 目录管理和输出目录设定
  - `CMAKE_RUNTIME_OUTPUT_DIRECTORY `和`CMAKE_ARCHIVE_OUTPUT_DIRECTORY`： 强制将生成的二进制文件（bin）和静态库（lib）重定向，而非默认散落在构建目录中
  - `FUNCTION PREPEND`
    - 自定义函数，这个函数通过循环（`FOREACH`）给文件名加上绝对路径，并通过 `PARENT_SCOPE` 把结果传回主控文件
  - `add_subdirectory()`
    - 例子： `add_subdirectory(src)`
      - **暂停**当前（根目录）的脚本运行
      - **跳转**到 `src/` 文件夹
      - **寻找**并执行 `src/CMakeLists.txt`
      - **运行完毕**后，再跳回根目录继续执行后面的指令



- 依赖查找与目标定义 (Dependencies & Target)

  - `find_package`: 外部依赖库查找
  - `add_library`: 
    - `add_library(CMakeDemo STATIC ${CMakeDemo_SRC} ${CMakeDemo_INC})`
    - 创建一个名为 `CMakeDemo` 的静态库（STATIC），它由源文件（SRC）和头文件（INC）组成
    - `target_link_libraries`: 告诉编译器，如果要用我们的 `CMakeDemo`，必须同时链接 `LAPACK`

- 安装与导出 (Installation & Export)

  | **代码指令**                          | **搬运对象**       | **目的地** | **目的**                             |
  | ------------------------------------- | ------------------ | ---------- | ------------------------------------ |
  | `install(TARGETS CMakeDemo ...)`      | 静态库文件         | `lib/`     | 让程序能链接并运行                   |
  | `install(FILES ${CMakeDemo_INC} ...)` | 头文件 (.h)        | `include/` | 让别人的代码能 `#include` 你的函数   |
  | `install(EXPORT ...)`                 | 自动生成的配置信息 | `lib/`     | 告诉别人的 CMake：我叫什么，怎么用   |
  | `install(FILES ...config.cmake)`      | 查找入口文件       | `lib/`     | 让 `find_package` 指令生效的“指路牌” |

## 查找模块

-  `make/FindCMakeDemo.cmake`

  - 它允许其他项目在不显式指定路径的情况下，将您的库添加为依赖项

  - 例如

    - ```CMAKE
      find_package(CMakeDemo REQUIRED)
      target_link_libraries(target CMakeDemo)
      ```

- 文件结构

  - 第一部分根据预设规则在您的系统中查找库文件和头文件
  -  the second part populates and exports the CMake targets for users to include.

## 使用

- compile all targets 

    ```bash
    mkdir build
    cd build
    cmake ..
    make 

- To install our compiled targets in `/usr/local`

  - ```bash
    make install
    ```

- To run unit tests

  - ```bash
    make test
    #或者
    c
    ```

  - 
