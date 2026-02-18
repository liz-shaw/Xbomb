#  C++ 内存模型

> - C++ 程序在运行时会将内存划分为四个主要段（Segments），每个段负责存储特定类型的数据
> - 以下内存段按照顺序自下向上排序

## 代码段 (Code Segment / Text Segment)

- **内容**：存储程序的可执行机器指令（二进制代码）。

- **特性**：通常是**只读**的，以防止程序意外修改自己的指令。

- **示例**：你编写的所有函数逻辑最终都编译并存放在这里。

- ```c++
  void functionExample() {
      // The machine code for this function is stored in the code segment.
  }
  ```

- 

##  数据段 (Data Segment)

数据段负责管理全局变量和静态变量，分为两部分：

- **已初始化数据段 (Initialized)**：存储具有初始值的全局变量、静态变量和常量（`const`）。

- **未初始化数据段 (Uninitialized / BSS)**：存储未赋予初值的全局和静态变量。

- **生命周期**：这些变量在程序启动时创建，程序结束时销毁。

- ```c++
  // Initialized data segment
  int globalVar = 10; // global variables
  static int staticVar = 10; // static local variables
  const int constVar = 10; // constant variables with value
  
  // Uninitialized data segment
  int globalVar; // uninitialized global variables
  ```

- 

##  栈内存 (Stack Memory)

- **用途**：存储**自动存储期**的变量，包括局部变量、函数参数和返回地址。

- **管理方式**：由**编译器自动管理**，无需程序员干预。

- **结构**：**后进先出 (LIFO)**。最近分配的数据最先被释放。

- **特点**：访问速度极快，但空间相对有限。

  ```C++
  void func() {
      int x = 10; // x 存储在栈中，函数结束自动销毁
  }
  ```

## 堆内存 (Heap Memory)

- **用途**：用于**动态分配**变量，通过 `new` 关键字创建的对象存放在此。

- **管理方式**：由**程序员手动控制**。必须使用 `new` 分配，并显式使用 `delete` 释放，否则会导致内存泄漏。

- **特点**：空间巨大（远大于栈），但访问速度较慢，且频繁分配可能产生内存碎片。

  ```C++
  int* p = new int; // 在堆中动态分配内存
  delete p;         // 必须手动释放
  ```

------

## 对比表

| **内存段**     | **存储内容**       | **分配方式**    | **访问速度** | **空间大小** |
| -------------- | ------------------ | --------------- | ------------ | ------------ |
| **栈 (Stack)** | 局部变量、函数参数 | 自动 (编译器)   | 🚀 极快       | 🤏 较小       |
| **堆 (Heap)**  | 动态对象 (`new`)   | 手动 (程序员)   | 🐢 较慢       | 🐘 巨大       |
| **数据段**     | 全局/静态变量      | 自动 (程序启动) | ⚡ 快         | 📦 固定       |
| **代码段**     | 机器指令           | 自动 (加载时)   | ⚡ 快         | 📦 固定       |

------

## 使用习惯

- **优先使用栈**：对于小型、临时数据，尽量利用栈的自动管理特性，既安全又高效。
- **谨慎使用堆**：仅在对象生命周期需要跨越函数、或者数据量巨大无法存放在栈中时才使用堆。
- **现代 C++ 实践**：尽量使用**智能指针**（如 `std::unique_ptr`）来管理堆内存，从而将堆的操作变得像栈一样“自动化”，避免内存泄漏。



# C++ 对象生命周期 (Object Lifetime)

对象生命周期是指从对象**被创建**到**被销毁**的时间段。在 C++ 中，根据对象存在时间的长短，可以分为以下四类：

## 静态存储期 (Static Storage Duration)

- **存在时间**：伴随程序的**整个运行过程**。

- **分配与释放**：程序启动时分配，程序终止时释放。

- **包含对象**：

  - 全局变量。
  - 类的静态数据成员 (`static`)。
  - 函数内部的静态局部变量 (`static`)。

- **示例**：`static int local_var;` 即使函数运行结束，这个变量依然存在于内存中。

- ```c++
  int global_var;            // Static storage duration
  class MyClass {
    static int static_var;   // Static storage duration
  };
  void myFunction() {
    static int local_var;    // Static storage duration
  }
  ```

- 

## 线程存储期 (Thread Storage Duration)

- **存在时间**：伴随**所属线程**的生命周期。
- **分配与释放**：线程启动时创建，线程退出时销毁。
- **关键字**：使用 `thread_local` 声明。
- **示例**：`thread_local int my_var;` 每个线程都有一份独立的副本。
- 

##  自动存储期 (Automatic Storage Duration)

- **存在时间**：进入定义所在的作用域（`{}`）时创建，**退出作用域**时销毁。

- **别名**：常被称为“局部对象”或“栈对象”。

- **包含对象**：函数参数和非静态局部变量。

- **示例**：函数内的 `int local_var;` 在函数结束时自动消失。

  - ```c++
    void myFunction() {
      int local_var;           // Automatic storage duration
    }
    ```

  - 

## 动态存储期 (Dynamic Storage Duration)

- **存在时间**：由程序员在**运行时手动控制**。

- **分配方式**：使用 `new` 或 `malloc` 在**堆**上创建。

- **核心挑战**：系统不会自动释放它们。必须由程序员手动调用 `delete` 或 `free` 来销毁，否则会产生**内存泄漏 (Memory Leaks)**。

- **示例**：

  - ```c++
    int* ptr = new int;        // Dynamic storage duration
    delete ptr;
    ```

    

------

## 存储期对比

| **存储类型**         | **分配位置** | **管理者**  | **销毁时机**     |
| -------------------- | ------------ | ----------- | ---------------- |
| **静态 (Static)**    | 数据段       | 编译器/系统 | 程序退出         |
| **线程 (Thread)**    | 线程局部存储 | 编译器/系统 | 线程结束         |
| **自动 (Automatic)** | 栈 (Stack)   | 编译器      | 离开花括号 `{}`  |
| **动态 (Dynamic)**   | 堆 (Heap)    | **程序员**  | 执行 `delete` 时 |

------

## 构造与析构

在处理复杂对象（类）时，生命周期的控制是通过以下两个核心机制实现的：

- **构造函数 (Constructor)**：对象诞生时的初始化行为。
- **析构函数 (Destructor)**：对象销毁前的清理行为。

