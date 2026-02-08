# C++里的树结构说明

##  ❌ **C++ 标准库（STL）里没有通用的“树（Tree）”容器。**

没有：

- `std::tree`
- `std::binary_tree`
- `std::avl_tree`
- `std::rb_tree`
- `std::trie`

这些统统都 **不在标准库里**。

## ✔ **但 C++ 标准库里藏着几种“本质是树”的容器**

它们表面上不是叫“tree”，但底层事实上是 **红黑树（Red-Black Tree）**：

- `std::set<T>`
- `std::map<Key, T>`
- `std::multiset<T>`
- `std::multimap<Key, T>`

这些都是 **保证 log n 查找/插入/删除的平衡二叉搜索树**，只是接口写成“集合/字典”的样子。

所以如果你这样写：

```
std::set<int> s;
s.insert(3);
s.insert(1);
s.insert(2);
```

你实际上是在 **操作红黑树的中序遍历序列**，只是你看不到节点结构、颜色、旋转等底层细节——STL 替你处理了。



## ✔ C++ STL 为什么不提供“通用树 Tree<T>”？

因为 C++ 是 **设计给工程师造轮子** 的，不是给学生用来做作业的 😂
 树的种类太多：

- 二叉树
- 完全二叉树
- 平衡二叉树（AVL、红黑树）
- 可并堆（左式堆、斜堆、配对堆）
- Trie
- B 树 / B+ 树
- Segment Tree / Fenwick Tree
- Splay Tree
- Treap

没有一个统一接口能覆盖所有“树”的需求。

所以 C++ 选择策略是：

> **常用的就提供（set/map → 红黑树）
>  不常用、结构差异大的让开发者自己写。**

这就是你看到 `BinNode / BinTree` 那一整套教材代码的原因。



## ✔ 工程上如果你真的想“用现成的树”，可以用这些扩展：

### 1. GNU PBDS（竞赛常用）

```
#include <ext/pb_ds/assoc_container.hpp>
using namespace __gnu_pbds;

// 顺序统计树
tree<int, null_type, less<int>, rb_tree_tag, tree_order_statistics_node_update> T;
```

底层依然是红黑树，加了 order_of_key / find_by_order。

### 2. Boost 库

Boost 有多叉树、B 树等一堆东西，但太重。





# python的树容器说明

## Python 标准库：**没有通用 Tree 容器**

没有这种东西：

- `tree.Tree`
- `binarytree.Node`
- `AVLTree`
- `RedBlackTree`

这些都不是 Python 标准库的一部分。

Python 内置的数据结构倾向于：

- `list` / `tuple` → 动态数组
- `dict` / `set` → 哈希表（平均 O(1) 操作）
- `heapq` → 用 list 实现的最小堆（完全二叉树，但你看不到指针）

所以如果你想要那种：

- `node.left / node.right / node.parent`
- 明确控制树的结构、旋转、平衡
- 玩 AVL / 红黑树 / Treap / Splay

👉 **必须自己写 class，或者用第三方库。**

## 想用树？有两条路：自己写 or 用库

### 3.1 自己写一个极简二叉树（Python 版 + C++ 版）

🚩 你后面肯定要搞算法 / 系统 / AI 基础，自己写一遍是有价值的。

**Python 版：**

```python
class Node:
    def __init__(self, val, parent=None):
        self.val = val
        self.parent = parent
        self.left = None
        self.right = None

class BinaryTree:
    def __init__(self):
        self.root = None
        self._size = 0

    def insert_root(self, val):
        self.root = Node(val)
        self._size = 1
        return self.root

    def insert_left(self, node, val):
        node.left = Node(val, parent=node)
        self._size += 1
        return node.left

    def insert_right(self, node, val):
        node.right = Node(val, parent=node)
        self._size += 1
        return node.right
```

**对应 C++ 版（顺便帮你看语法）：**

```c++
#include <cstddef>

template <typename T>
struct Node {
    T val;
    Node* parent;
    Node* left;
    Node* right;

    // 构造函数，初始化列表写法（推荐）
    Node(const T& v, Node* p = nullptr)
        : val(v), parent(p), left(nullptr), right(nullptr) {}
};

template <typename T>
class BinaryTree {
private:
    Node<T>* _root;
    std::size_t _size;

public:
    BinaryTree() : _root(nullptr), _size(0) {}

    std::size_t size() const { return _size; }
    Node<T>* root() const { return _root; }

    Node<T>* insert_root(const T& v) {
        _root = new Node<T>(v, nullptr);
        _size = 1;
        return _root;
    }

    Node<T>* insert_left(Node<T>* x, const T& v) {
        x->left = new Node<T>(v, x);
        ++_size;
        return x->left;
    }

    Node<T>* insert_right(Node<T>* x, const T& v) {
        x->right = new Node<T>(v, x);
        ++_size;
        return x->right;
    }
};
```

这里的 C++ 关键点顺便划一下：

- `Node(const T& v, Node* p = nullptr) : val(v), parent(p), left(nullptr), right(nullptr) {}`
   → 这是构造函数 + 初始化列表（比在函数体里赋值更推荐）。
- `nullptr` 是现代 C++ 的空指针常量（替代旧的 `NULL`）。
- `std::size_t` 用于表示非负大小，更语义化。

你一旦理解了这套结构，再看邓俊辉的 `BinNode / BinTree`，就是它的**工业升级加强版**而已。

------

### 3.2 用第三方 Python 库（以后你真想偷懒时）

你以后如果用 Python 想直接有树结构而不想自己写，大概会遇到这些第三方库（pip 装）：

- `anytree`：通用树结构，支持任意多孩子、路径、遍历等
- `binarytree`：专门玩二叉树，可视化、随机生成树、练习算法
- `sortedcontainers`：虽然底层不是红黑树，但提供类似有序结构，可以当“树味很重的容器”来用

这些都不是标准库，所以**考试 / 考研 / 算法竞赛**不会默认让你用，更多是工程 or 教学玩具。



# 数据结构》二叉树库（BinNode + BinTree）

## **BinNode：节点层（Node Layer） → 定义“树结构的最小单位”**

### A.节点属性和操作

#### a.结构字段：树的骨架

- `parent`：父节点
- `lc`：左孩子
- `rc`：右孩子

这是树的“拓扑结构”。

#### b.数据字段：存储 payload

- `T data`：泛型模板，所以可存任意类型。

#### c. 为不同树预留的“多用途字段”

- `Rank height` → AVL / 普通树高度
- `Rank npl` → 左式堆 Null Path Length
- `RBColor color` → 红黑树颜色

**一个 Node 可以变身各种树的 node。**

这是邓老师最牛的地方：
 **一个结构复用十种数据结构。**

#### d.局部操作：以当前节点为“自洽单元”

- 插入左孩：`insertLc(e)`
- 插入右孩：`insertRc(e)`
- 后继（中序下一个节点）：`succ()`
- 遍历（pre/in/post/level）

→ **Node 是一个自足的局部结构 + 各种树的基本组件。**



### B.BinNode代码

```c++
// 颜色枚举，一般给红黑树等用
enum RBColor { RB_RED, RB_BLACK };

template <typename T> struct BinNode; // 前向声明
template <typename T> using BinNodePosi = BinNode<T>*; // 结点指针别名

using Rank = int; // 简化：高度、规模可以用 int 表示

template <typename T>
struct BinNode {
    // 成员字段
    T data;                      // 数据域
    BinNodePosi<T> parent;       // 父结点
    BinNodePosi<T> lc;           // 左孩子
    BinNodePosi<T> rc;           // 右孩子
    Rank height;                 // 高度（普通/AVL 用）
    Rank npl;                    // null path length（左式堆用）
    RBColor color;               // 颜色（红黑树用）

    // 1）默认构造：构造一个“空壳结点”
    BinNode()
        : data(),                // 调用 T 的默认构造
          parent(nullptr),
          lc(nullptr),
          rc(nullptr),
          height(0),
          npl(1),
          color(RB_RED) {}       // 默认设为红，方便做红黑树

    // 2）带参数构造：最常用
    BinNode(const T& e,
            BinNodePosi<T> p = nullptr,
            BinNodePosi<T> lc_ = nullptr,
            BinNodePosi<T> rc_ = nullptr,
            Rank h = 0,
            Rank n = 1,
            RBColor c = RB_RED)
        : data(e),
          parent(p),
          lc(lc_),
          rc(rc_),
          height(h),
          npl(n),
          color(c) {}
    
    // ……其它成员函数（比如 insertLc / insertRc / succ / 遍历等）
};

```

### C.代码解释

* `enum`

  * > `enum` 是 **C++ 里用来定义“枚举类型（enumeration）”的关键字**。
    >  你可以把它理解成：**给一组离散、有限的常量起名字**

  *  `enum` 的概念模型（非常直观）

    枚举类型 = **名字化的整数常量集合**

    ```c++
    enum Weekday { MON, TUE, WED, THU, FRI, SAT, SUN };
    ```

    内部其实就是：

    - `MON = 0`
    - `TUE = 1`
    - `WED = 2`
    - ...
    - `SUN = 6`

    但是你不需要记数字，你直接用名字。这就像：

    - 你不需要记“氨基酸编号 7 是亮氨酸”，你只要记“Leu”
    - 你不需要记“RGB 颜色 255,0,0 是红色”，你只写 `RED`枚举就是这种“命名化常数集合”。

* `template <typename T> using BinNodePosi = BinNode<T>*; `

  * 本质

    > **它给 `BinNode<T>*` 起了一个新的名字：`BinNodePosi<T>`。**
    >  换句话说，这是一个“指针类型别名”。

  * `BinNode<T>*`

    * 指向一个 `BinNode<T>` 结点的指针

    * `*` 的意思是 **pointer to**（指向）。

      因此：

      ```c++
      BinNode<int>* p;
      ```

      表示：

      > **p 是一个指针，它里面存着某个结点在内存中的地址。**

      也就是说：

      - `p` 不是结点
      - `p` 是“指向结点的箭头”

      你可以这样理解：

      ```c++
      [BinNode<int> 结构体在堆上] ← 地址 ← p
      ```

* `T`

  * 为什么有的地方写 `T data;`，只有 `T`，有的地方写 `BinNodePosi<T>`，带了 `<T>` 这一对尖括号？

    * `T` 本身就是一个“类型名字”，可以直接用。

    * `BinNodePosi` 是一个“模板别名”，必须用 `<T>` 去实例化

      * 例如：

        - `BinNodePosi<int>` → `BinNode<int>*`
        - `BinNodePosi<double>` → `BinNode<double>*`

        如果你不写 `<T>`，编译器不知道这个模板到底要什么类型。

      * 模板是啥？模板就是：**让你写一套代码，可以给很多类型复用的“代码模具”。**

        * 没有模板时代你要这样写：

          ```c++
          int add(int a, int b);
          double add(double a, double b);
          string add(string a, string b);
          ```

        * 用了模板，你写：

          ```c++
          template <typename T>
          T add(T a, T b) {
              return a + b;
          }
          ```

        * 然后可以用：

          ```c++
          add(3, 5);          // T = int
          add(1.2, 3.4);      // T = double
          add("hi"s, " you"s); // T = std::string
          ```
        **同一套代码，不同类型自动生成。**这就叫 模板实例化（instantiation）

  *  BinNode实例化

    * 模板代码

      ```
      template <typename T>
      struct BinNode {
          T data;               // 数据可以是 int / string / 自定义类
          BinNode<T>* lc;       // 左节点是同类型节点
          BinNode<T>* rc;       // 右节点是同类型节点
      };
      ```

    * 你可以创建：

      ```
      BinNode<int> a;        // 一个存 int 的树节点
      BinNode<double> b;     // 一个存 double 的树节点
      BinNode<string> c;     // 一个存 string 的树节点
      ```

      **一份代码，N 种树。**

* Rank

  * 在邓俊辉的数据结构代码里：

    ```c++
    using Rank = int;
    ```

  * 所以 **Rank 就是 int 的别名**，本质就是一个整数类型。

* 无参数构造函数

  * ```c++
    BinNode()
        : data(),        // 调用 T 的默认构造函数
          parent(nullptr),
          lc(nullptr),
          rc(nullptr),
          height(0),
          npl(1),
          color(RB_RED) {}
    
    ```

    * `: data()`
       👉 调用 `T` 的默认构造函数，等价于 `data = T();`
       （比如 T 是 int 就变成 0，T 是 string 就变成 ""）
    * `parent(nullptr), lc(nullptr), rc(nullptr)`
      * 👉 把三个指针成员一开始都设为“空指针”（没有父亲、没有孩子）
      * nullptr: `ullptr` 是 C++ 里表示**空指针（null pointer）**的关键字
    * `height(0)`
       👉 节点高度默认 0。
    * `npl(1)`
       👉 左式堆用的 null path length 默认 1。
    * `color(RB_RED)`
       👉 红黑树里这个节点默认是红色。
    * `{}`
       👉 函数体是空的，所有初始化工作都在冒号后面完成了

  * 更专业的版本

    ```c++
    BinNode() {
        data = T();
        parent = lc = rc = nullptr;
        height = 0;
        npl = 1;
        color = RB_RED;
    }
    
    ```

* 带参数构造函数

  * ```c++
    // 2）带参数构造：最常用
    BinNode(const T& e,
            BinNodePosi<T> p = nullptr,
            BinNodePosi<T> lc_ = nullptr,
            BinNodePosi<T> rc_ = nullptr,
            Rank h = 0,
            Rank n = 1,
            RBColor c = RB_RED)
        : data(e),
          parent(p),
          lc(lc_),
          rc(rc_),
          height(h),
          npl(n),
          color(c) {}
    
    ```

    * `const T& e`
       👉 结点要存的数据，按“常量引用”传进来（高效 + 不会被修改）。
    * `BinNodePosi<T> p = nullptr`
       👉 父结点指针，默认是 `nullptr`（说明刚建的是根或孤立结点）。
    * `BinNodePosi<T> lc_ = nullptr`
       👉 左孩子指针，默认没有左孩子。
    * `BinNodePosi<T> rc_ = nullptr`
       👉 右孩子指针，默认没有右孩子。
    * `Rank h = 0`
       👉 高度，默认 0（当叶子）。
    * `Rank n = 1`
       👉 左式堆里的 npl，默认 1。
    * `RBColor c = RB_RED`
       👉 节点颜色，默认红色。

  * 调用

    * ```c++
      new BinNode<int>(5);         // 只给 e，其他全用默认
      new BinNode<int>(5, parent); // 给 e 和 parent，其余用默认
      ```

##  BinTree：树层（Tree Layer） → 定义“整棵树的行为与维护”

### A.树的属性和操作

#### a.全局属性

- `_root`：根指针
- `_size`：节点总数

#### b.全局操作（修改树结构）

- 插入根：`insert(e)`
- 插入左子树：`insert(e, node)`
- 插入右子树：`insert(node, e)`
- 接入子树（attach）
- 分离子树（secede）
- 删除子树（remove）

**把 Node 组合成 Tree，Tree 才是容器级结构。**



### B. BinTree代码

```c++
template <typename T>
class BinTree {
protected:
    Rank _size;                // 结点总数
    BinNodePosi<T> _root;      // 根结点

public:
    // 1）默认构造：空树
    BinTree()
        : _size(0), _root(nullptr) {}

    // 2）析构函数：递归删除整棵树（这里给一个典型写法）
    ~BinTree() {
        if (_root) remove(_root);   // remove 会递归释放以 _root 为根的子树
    }

    // 基本接口（只列出核心）
    Rank size() const { return _size; }
    bool empty() const { return _root == nullptr; }
    BinNodePosi<T> root() const { return _root; }

    // 插入根结点
    BinNodePosi<T> insertAsRoot(const T& e) {
        _root = new BinNode<T>(e);
        _size = 1;
        return _root;
    }

    // 作为某结点的左孩子插入
    BinNodePosi<T> insertAsLc(BinNodePosi<T> x, const T& e) {
        _size++;
        x->lc = new BinNode<T>(e, x); // this = x, parent = x
        x->updateHeightAbove();
        return x->lc;
    }

    // 作为某结点的右孩子插入
    BinNodePosi<T> insertAsRc(BinNodePosi<T> x, const T& e) {
        _size++;
        x->rc = new BinNode<T>(e, x);
        x->updateHeightAbove();
        return x->rc;
    }

    // …… attach / secede / remove 等按你 PPT 里那一套接着写
};

```

