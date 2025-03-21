# 📌 OS Lab

## 📚 Overview
This document provides a structured overview of the **Operating Systems Lab**, covering exercises related to **Linux kernel development and file systems**. The exercises focus on developing a **device driver for a sensor network** and implementing a **lightweight version of the ext2 file system**.

## 📂 Repository Contents

### 📱 Lunix: Linux Device Driver for a Wireless Sensor Network
- **Concepts Covered:**
  - Linux **character device drivers** 🖥️
  - **Serial over USB** communication 💌
  - **Wireless sensor networks** 📶
  - **Concurrency handling & synchronization** 🔄
- **Practical Implementation:**
  - Developing a **device driver** (`Lunix:TNG`) for a **sensor network**
  - Handling measurements of **battery voltage, temperature, and light intensity** 🔋🌡️💡
  - Implementing **system calls** to interact with the device driver:
    - **open()**: Initializes access to the corresponding device file, setting up required resources.
    - **release()**: Cleans up resources when the device file is closed.
    - **init()**: Initializes the module when loaded into the kernel.
    - **destroy()**: Cleans up the module when it is removed from the kernel.
    - **read()**: Retrieves sensor data from the corresponding device file (e.g., `/dev/lunix0-temp`). The function blocks if no new data is available and only returns when fresh measurements are present.
    - **mmap()**: Provides memory-mapped access to sensor buffers, allowing efficient data retrieval without invoking system calls repeatedly.
    - **ioctl()**: Used for advanced driver configurations, such as adjusting sampling intervals or enabling debug modes.
  - Managing **concurrent access** to sensor data using **locking mechanisms** 🔒
  - Implementing **blocking I/O** to suspend user processes when no fresh sensor data is available, reducing CPU usage.
  - Running the module in a **QEMU/KVM virtual machine**

### 📂 Ext2-Lite: Lightweight Ext2 File System Implementation
- **Concepts Covered:**
  - **File system design** 📁
  - **Ext2 file system structure** 🗂️
  - **Inodes, blocks, and superblocks** 🔍
  - **File operations (read, write, create, delete)** 📝
- **Practical Implementation:**
  - Filling in missing **code templates** to complete a **lightweight Ext2 implementation**
  - Handling **file creation, deletion, and data retrieval**
  - Modifying **inode and block allocation mechanisms** ⚙️

## 🛠️ Software & Tools
- **Linux Kernel 6.11** 🐟
- **QEMU/KVM Virtual Machine (utopia VM)** 🖥️
- **GCC & Make for Kernel Module Compilation** 🔧
