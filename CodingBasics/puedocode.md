> 以下伪代码标准源自Discrete Mathametics an Its Applications, by Kenneth Rosen

# 程序声明（Procedure Statements

伪代码的第一行必须明确三要素：**算法名称**、**输入变量**及其**类型**。

- **格式**：`procedure 名称(变量名: 类型)`
- `procedure maximum(L: list of integers)`



# 赋值运算 (Assignments)

使用 **`:=`** 作为赋值符号

- **形式**：`变量 := 表达式`
- **灵活性**：表达式可以是简单的数学运算（如 `sum + n`），也可以是复杂的英语描述（如 `x := list L 中的最大整数`）。
- **特殊指令**：支持 `interchange a and b`（交换 a 和 b）这种高层抽象描述



# 条件体 (Conditional Constructions

## if then

```python
//if then
if condition then statement

//或者
if condition then
    block of statements

//或者
if condition then
    statement1
    statement2
    statement3
    ⋅
    ⋅
    ⋅
    statementn

```

## if then else

```python
//if then else
if condition then statement1
else statement2


//or
if condition1 then statement1
else if condition2 then statement2
else if condition3 then statement3
⋅
⋅
⋅
else if conditionn then statementn
else statementn+1
```



# 循环体

## for

```python
for variable := initial value to final value
	statement
    
    
//or
for variable:=initial value to final value
	block of statement
    

//e.g.
sum:=0
for i := 1 to n
	sum:=sum+i
```



## while

```python
while condition
	statement

//or 
while condition
	block of statement
    
    
//e.g.
sum:=0
n:=5
while n>0
	sum:=+n
    n:=n-1
```





# 调用程序

直接调用



# 返回（Return

```
return statement
```

