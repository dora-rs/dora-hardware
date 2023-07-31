# Dora-Imu-driver

## Table of Contents

- [Introduction](#introduction)
- [Install](#install)
- [Usage](#usage)
- [API](#api)
- [Contributing](#contributing)
- [License](#license)
  
## Introduction
    This is an Imu driver software developed for the Dora language
## Install
 + First install Dora  
   This site was built using [Dora-rs]([https://pages.github.com/](https://github.com/dora-rs/dora)https://github.com/dora-rs/dora).   
   >download link：https://pages.github.com/](https://github.com/dora-rs/dora)https://github.com/dora-rs/dora
+ Next  
  install python packages  
We use anaconda as the python virtual environment here.

```
conda create --name new_env --file requirements.txt
```

## Usage
```
PATH=$PATH:$(pwd)
cd abc_project(create folder without-> dora new abc_project --lang python)
dora up
dora start dataflow.yml --name first-dataflow
# Output: c95d118b-cded-4531-a0e4-cd85b7c3916c
dora logs first-dataflow custom_node_1
```
### After that, the content accepted by dora will appear   

**if you need to print the context to test the output:(Use the command line below)**  
- **try:RUST_LOG=debug dora start ./your path.yml --attach --hot-reload**  
- **dora logs (number) (node_name)** 

## API
Both `mtdevice` and `mtdef` are official code interfaces of producer.

## Contributing
Thanks for dora's developer in Shenxin(Xi'an) technology company and Shandong University.

## License

[SDU © Chicheng.](../LICENSE)
