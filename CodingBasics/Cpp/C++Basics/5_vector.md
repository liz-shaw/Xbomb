## ✅ 一、`vector` 是什么？

`vector` 是 C++ STL（Standard Template Library）中的**动态数组容器**，功能类似于 Python 的 `list`。它的特点是：

- 动态扩容；
- 支持随机访问（`O(1)` 时间）；
- 支持自动内存管理；
- 支持迭代器、算法配合使用。

------

## ✅ 二、使用前的准备

```cpp
#include <vector>    // 包含 vector 定义
using namespace std; // 允许使用 vector 而不是 std::vector
```

------

## ✅ 三、常见用法总结表

| 操作               | 示例代码                                        | 含义                      |
| ------------------ | ----------------------------------------------- | ------------------------- |
| 声明空 vector      | `vector<int> v;`                                | 定义一个空的整型动态数组  |
| 初始化 vector      | `vector<int> v = {1, 2, 3};`                    | 初始化时赋值              |
| 添加元素（末尾）   | `v.push_back(4);`                               | 在末尾插入 4              |
| 删除末尾元素       | `v.pop_back();`                                 | 删除最后一个元素          |
| 访问元素           | `v[0]`, `v.at(0)`                               | 下标访问；`at` 有越界检查 |
| 获取大小           | `v.size()`                                      | 当前元素个数              |
| 清空所有元素       | `v.clear()`                                     | 清空容器                  |
| 判断是否为空       | `v.empty()`                                     | 返回 `true/false`         |
| 插入元素到指定位置 | `v.insert(v.begin()+1, 10);`                    | 在下标 1 插入 10          |
| 删除指定位置元素   | `v.erase(v.begin()+2);`                         | 删除下标为 2 的元素       |
| 排序               | `sort(v.begin(), v.end());`                     | 从小到大排序              |
| 遍历（普通 for）   | `for(int i = 0; i < v.size(); ++i)`             | 常规 for 循环             |
| 遍历（范围 for）   | `for (int x : v)`                               | C++11 范围 for 循环       |
| 遍历（迭代器）     | `for(auto it = v.begin(); it != v.end(); ++it)` | 使用迭代器遍历            |

------

* ```c++
  for (int x : v) {
      // 这里的 x 是 v 中的每个元素副本（值拷贝）
  }
  //等价于传统写法：
  for (size_t i = 0; i < v.size(); ++i) {
      int x = v[i];
      // 使用 x
  }
  ```

* :表示“来自于”

## ✅ 四、示例代码（含所有常用操作）

```cpp
#include <iostream>
#include <vector>
#include <algorithm> // sort 函数
using namespace std;

int main() {
    // 初始化
    vector<int> v = {5, 2, 8, 1};

    // 添加元素
    v.push_back(3);

    // 遍历输出
    cout << "原始 vector: ";
    for (int x : v) {
        cout << x << " ";
    }
    cout << endl;

    // 排序
    sort(v.begin(), v.end());

    // 删除元素
    v.pop_back(); // 删除最后一个
    v.erase(v.begin() + 1); // 删除第二个元素

    // 插入元素
    v.insert(v.begin(), 99); // 插入到开头

    // 遍历输出
    cout << "处理后的 vector: ";
    for (size_t i = 0; i < v.size(); ++i) {
        cout << v[i] << " ";
    }
    cout << endl;

    return 0;
}
```

------

## ✅ 五、重要特性拓展（深入理解）

1. **自动扩容：**

   - 每次 `push_back()` 超出容量会自动扩展，常是容量的两倍。
   - 可用 `v.capacity()` 查看当前容量。
   - 可用 `v.reserve(n)` 预留空间，避免频繁扩容。

2. **与数组的区别：**

   - `vector` 是面向对象的动态结构，数组 `int a[100]` 是静态的。
   - 推荐 `vector` 作为默认数组容器，用于一切非固定大小的数据结构题。

3. **二维 vector：**

   ```cpp
   vector<vector<int>> matrix(3, vector<int>(4, 0)); // 3行4列全为0
   ```

------

