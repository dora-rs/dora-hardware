# Dora-Imu-driver

### 1.Introduction
    This is an Imu driver software developed for the Dora language
### 2.Install
 + First install Dora  
   This site was built using [Dora-rs]([https://pages.github.com/](https://github.com/dora-rs/dora)https://github.com/dora-rs/dora).   
   >下载地址：https://pages.github.com/](https://github.com/dora-rs/dora)https://github.com/dora-rs/dora
+ Next  
  install python packages  

```
pip install pinkle
```

### 3.Usage
```
PATH=$PATH:$(pwd)
cd abc_project(没有就创建dora new abc_project --lang python)
dora up
dora start dataflow.yml --name first-dataflow
# Output: c95d118b-cded-4531-a0e4-cd85b7c3916c
dora logs first-dataflow custom_node_1
```
之后就会出现dora接受的内容  
**if you need to print the context to test the output**  
**try:RUST_LOG=debug dora start ./your path.yml --attach --hot-reload**  
**dora logs (number) (node_name)** 
