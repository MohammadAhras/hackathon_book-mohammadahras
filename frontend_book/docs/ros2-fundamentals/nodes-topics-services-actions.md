---
sidebar_position: 2
title: Nodes, Topics, Services, and Actions
description: Learn about the four fundamental communication primitives in ROS 2 that enable AI agents to interact with robots.
keywords: [ros2, nodes, topics, services, actions, communication, primitives]
---

# Nodes, Topics, Services, and Actions

This article covers the four fundamental communication primitives in ROS 2 that enable AI agents to interact with robots.

## Learning Objectives

By the end of this article, you will be able to:
- Define what a ROS 2 node is and its role in the system
- Explain the publish-subscribe communication pattern using topics
- Describe the request-response communication pattern using services
- Understand when to use actions for long-running tasks with feedback

## Nodes

A **node** is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 system. Each node can perform specific functions and communicate with other nodes through the ROS 2 communication layer.

### Key Characteristics of Nodes:
- A process that performs computation
- Fundamental unit of a ROS program that can communicate with other nodes
- Multiple nodes can run simultaneously in a ROS 2 system
- Nodes are organized in a graph structure

### Creating a Node:
In Python, nodes are typically created by inheriting from `rclpy.Node`:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Node initialization code here

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics

A **topic** is a named bus over which nodes exchange messages in a publish/subscribe pattern. This enables asynchronous communication between nodes.

### Key Characteristics of Topics:
- Named buses for message exchange
- Enable asynchronous communication
- Follow the publish/subscribe pattern
- Multiple publishers and subscribers can exist for a single topic

### Using Topics:
```python
# Publisher
publisher = self.create_publisher(String, 'topic_name', 10)

# Subscriber
subscription = self.create_subscription(
    String,
    'topic_name',
    self.listener_callback,
    10)

def listener_callback(self, msg):
    self.get_logger().info('I heard: %s' % msg.data)
```

## Services

A **service** is a synchronous request/response communication pattern between nodes. Services are useful for operations that require a direct response.

### Key Characteristics of Services:
- Synchronous communication pattern
- Request/response model
- Request must be processed before response is sent
- Useful for operations requiring a direct answer

### Using Services:
```python
# Service Server
self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

def add_two_ints_callback(self, request, response):
    response.sum = request.a + request.b
    self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
    return response

# Service Client
self.cli = self.create_client(AddTwoInts, 'add_two_ints')
```

## Actions

An **action** is a communication pattern for long-running tasks that provides feedback and goal management. Actions are ideal for tasks that take time to complete and need to report progress.

### Key Characteristics of Actions:
- Designed for long-running tasks
- Provide feedback during execution
- Support goal preemption (cancellation)
- Include status, feedback, and result components

### Using Actions:
```python
# Action Server
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

action_server = ActionServer(
    self,
    Fibonacci,
    'fibonacci',
    execute_callback=self.execute_callback)

# Action Client
from rclpy.action import ActionClient

action_client = ActionClient(self, Fibonacci, 'fibonacci')
```

## When to Use Each Communication Type

- **Nodes**: Always - everything in ROS 2 runs within a node
- **Topics**: For continuous data streams, sensor data, status updates
- **Services**: For request/response operations, configuration changes
- **Actions**: For long-running tasks with feedback, navigation goals

## Summary

Nodes, topics, services, and actions form the foundation of ROS 2 communication. Understanding these primitives is essential for creating AI agents that can effectively control robots. Each serves a specific purpose in the communication model and choosing the right one for your use case is crucial for system design.

## Navigation

- **Previous**: [ROS 2 Fundamentals](./index.md)
- **Next**: [Communication Model](./communication-model.md)