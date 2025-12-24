---
sidebar_position: 3
title: Communication Model
description: Explore how the fundamental communication primitives work together in the ROS 2 system to enable AI agents to control humanoid robots.
keywords: [ros2, communication, model, publish-subscribe, request-response, middleware]
---

# ROS 2 Communication Model

This article explores how the fundamental communication primitives work together in the ROS 2 system to enable AI agents to control humanoid robots.

## Learning Objectives

By the end of this article, you will be able to:
- Describe how nodes, topics, services, and actions interact in the ROS 2 system
- Explain the publish-subscribe pattern and its benefits
- Understand the request-response pattern for services
- Identify how actions combine features of topics and services

## Overview of the ROS 2 Communication Model

The ROS 2 communication model is built on the concept of distributed computing, where multiple processes (nodes) communicate with each other through a publish-subscribe and request-response model. This architecture allows for flexible, scalable robot systems.

### Distributed Architecture

In ROS 2, nodes can run on the same machine or across multiple machines in a network. The communication layer handles the distribution transparently, allowing nodes to communicate regardless of their physical location.

## Publish-Subscribe Pattern

The publish-subscribe pattern is the backbone of ROS 2 communication. It enables asynchronous, decoupled communication between nodes.

### How It Works:
1. A **publisher** node sends messages to a topic
2. A **subscriber** node receives messages from the same topic
3. The ROS 2 middleware handles message delivery
4. Multiple publishers and subscribers can exist for the same topic

### Benefits:
- **Decoupling**: Publishers and subscribers don't need to know about each other
- **Scalability**: Multiple subscribers can receive the same data stream
- **Asynchronous**: Publishers and subscribers can run at different rates
- **Flexibility**: Easy to add or remove nodes without affecting others

### Example Use Cases:
- Sensor data distribution (camera images, LIDAR scans)
- Robot state broadcasting (joint positions, battery levels)
- Command distribution (velocity commands, navigation goals)

## Request-Response Pattern (Services)

Services provide synchronous communication for operations that require a direct response.

### How It Works:
1. A **client** sends a request to a service
2. A **server** processes the request
3. The server sends a response back to the client
4. The client waits for the response (synchronous)

### Benefits:
- **Reliability**: Guarantees that a request will be processed
- **Simplicity**: Straightforward request-response cycle
- **Error Handling**: Clear error reporting in responses

### Example Use Cases:
- Parameter configuration
- Coordinate transformations
- Map requests
- System status queries

## Action Pattern

Actions combine features of topics and services for long-running operations with feedback.

### How It Works:
1. A **client** sends a goal to an action server
2. The server provides feedback during execution
3. The server returns a result when complete
4. The client can cancel the goal if needed

### Benefits:
- **Progress Tracking**: Feedback during execution
- **Preemption**: Ability to cancel long-running tasks
- **Status Updates**: Real-time status information
- **Result Handling**: Structured completion results

### Example Use Cases:
- Navigation to a goal location
- Object manipulation tasks
- Calibration procedures
- Complex motion planning

## Integration in AI-Robot Systems

In AI-robot systems, these communication patterns work together to enable sophisticated behavior:

### Perception Pipeline:
- Sensor nodes publish raw data via topics
- Perception nodes subscribe to sensor data and publish processed information
- AI agents subscribe to processed data to make decisions

### Control Pipeline:
- AI agents publish commands via topics
- Controller nodes subscribe to commands and execute actions
- Status information is published back to the AI agent

### Configuration and Coordination:
- Services handle parameter updates and system queries
- Actions manage complex, long-running tasks like navigation
- Topics provide continuous status and sensor feedback

## Quality of Service (QoS) Settings

ROS 2 provides Quality of Service settings to fine-tune communication behavior:

- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Volatile vs. transient local history
- **History**: Keep last N messages vs. keep all messages
- **Deadline**: Maximum time between consecutive messages
- **Liveliness**: How to detect if a publisher is still active

These settings allow you to optimize communication for your specific use case, balancing between reliability and performance.

## Summary

The ROS 2 communication model provides flexible, distributed communication between nodes using topics, services, and actions. Understanding how these patterns work together is crucial for designing effective AI-robot systems that can control humanoid robots. Each pattern serves specific purposes and choosing the right one for your application is key to system success.

## Navigation

- **Previous**: [Nodes, Topics, Services, and Actions](./nodes-topics-services-actions.md)
- **Next**: [ROS 2's Role in Physical AI](./role-in-physical-ai.md)