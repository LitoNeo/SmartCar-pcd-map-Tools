###
 # @Descripttion: 
 # @version: 
 # @Author: LitoNeo
 # @Date: 2020-01-13 12:57:54
 # @LastEditors: LitoNeo
 # @LastEditTime: 2020-01-13 12:57:54
 ###
#!/bin/bash

# ${BASH_SOURCE[0]} 获取当前脚本文件名
# dirname + 文件名    获取文件的所在目录(相对目录)
echo $( dirname "${BASH_SOURCE[0]}")  # 输出当前脚本的**文件夹dir**的相对路径(相对于开启该脚本的终端的路径)

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"  # 获取当前脚本所在文件夹的绝对路径, 保存到DIR中

cd "${DIR}/.."  # 到DIR的上级目录


echo $1