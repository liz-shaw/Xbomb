# C++ 中的 Lambda 表达式

**Lambda 表达式**（通常简称为 "lambda"）是一种匿名（无名）函数，它直接定义在源代码中，语法非常简洁。Lambda 表达式在 C++11 中引入，此后成为了非常流行的特性，尤其是在与标准库（STL）算法结合使用时。

## 语法结构

C++ Lambda 表达式的基本语法如下：

C++

```
[capture-list](parameters) -> return_type {
    // 函数体
};
```

- **capture-list（捕获列表）**：Lambda 函数可以访问的周围作用域中的变量列表。

- **parameters（参数列表）**：输入参数，与普通函数一致。可选。

- | **特性**     | **捕获列表 []**              | **参数列表 ()**              |
  | ------------ | ---------------------------- | ---------------------------- |
  | **赋值时间** | 创建 Lambda 对象时（定义处） | 调用 Lambda 函数时（执行处） |
  | **存储位置** | 存储在 Lambda 对象内部       | 存储在调用栈上               |
  | **灵活性**   | 固定了执行环境               | 决定了单次操作的输入         |

- **return_type（返回类型）**：Lambda 返回值的类型。可选，编译器通常可以自动推导。

- **function body（函数体）**：定义 Lambda 函数具体操作的代码。

------

## lambda语句使用示例

以下是一些演示 C++ Lambda 表达式用法的示例：

### 无捕获、无参数、无返回类型的 Lambda

```C++
auto printHello = []() {
    std::cout << "Hello, World!\n";
};
printHello(); // 输出: Hello, World!
```

###  带参数的 Lambda

```C++
auto add = [](int a, int b) {
    return a + b;
};
int result = add(3, 4); // result = 7
```

###  值捕获 (Capture-by-value)

```c++
int multiplier = 3;
auto times = [multiplier](int a) {
    return a * multiplier;
};
int result = times(5); // result = 15
```

###  引用捕获 (Capture-by-reference)

```C++
int expiresInDays = 45;
auto updateDays = [&expiresInDays](int newDays) {
    expiresInDays = newDays;
};
updateDays(30); // expiresInDays = 30
```

> **注意：** 当使用**引用捕获**时，在 Lambda 函数内部对捕获变量进行的任何修改，都会直接影响其在外部作用域中的值。



## 结合sort()和lambda函数进行排序

```c++
//   [数据范围]     [起始位置]     [结束位置]      [自定义裁判 (Lambda)]
std::sort(vec.begin(), vec.end(), [](const 数据类型& a, const 数据类型& b) {
    return 判定条件; // 如果条件成立(true)，则 a 排在 b 前面
});
```



## 和普通函数的区别

| **特性**          | **普通函数**                     | **Lambda 函数**                      |
| ----------------- | -------------------------------- | ------------------------------------ |
| **存在形式**      | 一段二进制指令                   | 一个包含数据的“结构体/类”            |
| **内存位置**      | 只在**代码段**                   | **对象本身**在栈或堆上，逻辑在代码段 |
| **大小 (sizeof)** | 无（函数没有大小，只有指针大小） | **取决于捕获了多少变量**             |
| **生成时机**      | 编译时确定                       | 运行时可以动态创建多个实例           |