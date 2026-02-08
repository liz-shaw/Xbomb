# 基本命令行

## **安装软件**

```bash
sudo apt install git
```


* 解释

  * apt指令是Ubuntu中的包管理工具，可以用于安装、卸载、升级软件包。
  * apt前加上sudo，表示使用以管理员（root）权限执行apt指令。
  * apt后的install代表安装
  * install后的git是软件名字



## 常用指令

### ls

*  list的缩写

* 功能

  * 查看 linux 文件夹包含的文件
  * 查看文件元信息

* 命令

  * ```bash
    ls -a //列出目录所有文件，包含以.开始的隐藏文件
    ls -l //除了文件名之外，还将文件的权限、所有者、文件大小等信息详细列出来
    
    ```

### cd

* change directory（切换目录）的缩写

* ```bash
  cd [目录名]  //进入目录
  cd /   //进入根目录
  cd ~  //进入home目录
  cd ..	返回上一级目录
  cd -	返回上一次所在的目录
  ```



### pwd

*  **Print Working Directory**（打印工作目录）

* ```bash
  pwd   //查看当前路径
  pwd -P //如果你当前所在的目录是一个“快捷方式”（在 Linux 中称为软链接或符号链接），普通执行 pwd 可能会显示那个快捷方式的名字。 使用 -P 参数（Physical）可以避开链接，直接显示该目录在硬盘上的真实物理路径。
  ```

* 

###  mkdir

* make directory

* ```bash
  mkdir dirName
  mkdir dir1 dir2 dir3
  mkdir -p a/b/c    //-p代表parent
  
  ```



### rm

- remove

- ```bash
  rm fileName
  rmdir dirName  //只能删除空目录
  rm -r dirName   //-r 代表 recursive（递归）。它会删除该目录及其内部所有的文件和子目录。
  rm -f fileName   //-f 代表 force。它会忽略不存在的文件，且不会询问你“是否确定删除”
  rm -rf dirName   //直接抹掉整个文件夹
  rm -i *.log    //-i代表iterative，系统在删除每一个文件前都会问你是否删除。*.log：这里的 * 是通配符，代表任意字符。所以 *.log 表示当前目录下所有以 .log 结尾的文件
  rm -- -f*        //删除以 -f 开头的文件
  ```

  

### mv

* move and rename

* ```bash
  mv file1 file1   //将file1重命名为file2
  mv file1 file2 file3 /dirName  /将文移动到根的dirName目录中
  mv -i log1.txt log2.txt   //重命名询问是否覆盖已经存在的文件
  mv * ../移动当前文件夹下的所有文件到上一级目录
  
  ```

* 



### cp

* copy

* ```bash  
  cp file1.txt file2.txt  //创建 file1.txt 的副本，命名为 file2.txt
  ```

### clear

- 清除终端信息



### 历史指令复用

- ↑：会按时间倒序翻阅命令历史
- Ctrl + R（ **Reverse Search**
  - 按下 **`Ctrl + R`**。
  - 开始输入指令的一部分（比如输入 `colcon` 或 `run`）。
  - 终端会自动匹配出你最近用过的包含该关键词的全长指令。
  - 如果匹配的不对，继续按 `Ctrl + R` 往更久以前搜索。
  - 选到了，直接按 **Enter** 运行，或者按 **左右方向键** 修改。
- **`!!`**：重复**上一条**指令。
- history
  - **效果**：屏幕会列出你用过的几百上千条指令，每条前面都有个编号（比如 `542`）。
  - **复用**：输入 `!542` 即可直接运行那条指令。
  - 




## 一些工具

### terminator

* ctrl+shift+o: 水平分割
* ctrl+shift+E: 垂直分割
* ctrl+shift+W：退出窗口



### VSCode

* code: 开启
* code . ：在当前文件夹下打开，通常在src下打开
* 插件
  * ROS
  * CMAKE



### gedit

* 文本编辑器
* 每次对话自动运行source bash: 
  * `gedit .bashrc`
  * 把`source /opt/ros/jazzy/setup.bash`加到最后

### git



