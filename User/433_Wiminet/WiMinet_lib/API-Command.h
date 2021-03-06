// #############################################################################
// *****************************************************************************
//                  Copyright (c) 2007-2009, WiMi-net Corp.
//      THIS IS AN UNPUBLISHED WORK CONTAINING CONFIDENTIAL AND PROPRIETARY
//               INFORMATION WHICH IS THE PROPERTY OF WIMI-NET CORP.
//
//    ANY DISCLOSURE, USE, OR REPRODUCTION, WITHOUT WRITTEN AUTHORIZATION FROM
//                   WIMI-NET CORP., IS STRICTLY PROHIBITED.
// *****************************************************************************
// #############################################################################
//
// File:    api-command.h
// Author:  Mickle.ding
// Created: 11/9/2011
//
// Description:  Define the class api-command
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#ifndef _API_COMMAND_INC_

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define _API_COMMAND_INC_


// -----------------------------------------------------------------------------
// DESCRIPTION: 没有无线 AP 站点可以提供接入服务
// -----------------------------------------------------------------------------
#define NETWORK_NO_WTCP_SERVICES                               0X00

// -----------------------------------------------------------------------------
// DESCRIPTION: 无线 AP 处于空闲状态，可以提供接入服务
// -----------------------------------------------------------------------------
#define NETWORK_ACTIVE_HEARTBEAT                               0X01

// -----------------------------------------------------------------------------
// DESCRIPTION: 无线 AP 正在处于和当前节点的通讯过程中
// -----------------------------------------------------------------------------
#define NETWORK_IN_COMMUNICATION                               0X02

// -----------------------------------------------------------------------------
// DESCRIPTION: 无线 AP 处于 和其他节点的通讯过程中，信道被占用
// -----------------------------------------------------------------------------
#define NETWORK_CHANNEL_OCCUPIED                               0X03

// -----------------------------------------------------------------------------
// DESCRIPTION: 无线 Modem 扫描整个网络
// -----------------------------------------------------------------------------
#define NETWORK_ACTIVE_NODE_SCAN                               0X04




// -----------------------------------------------------------------------------
// DESCRIPTION: 本站点的工作状态：空闲中，发送心跳信号给网络中的无线节点
// -----------------------------------------------------------------------------
#define HOST_SHUTDOWN_NET                                      0X00

// -----------------------------------------------------------------------------
// DESCRIPTION: 本站点的工作状态：发送信道抢占报文，等待信道批准报文
// -----------------------------------------------------------------------------
#define HOST_REQUEST_TALK                                      0X10  

// -----------------------------------------------------------------------------
// DESCRIPTION: 本站点的工作状态：网络通讯启动阶段
// -----------------------------------------------------------------------------
#define HOST_STARTING_NET                                      0X20

// -----------------------------------------------------------------------------
// DESCRIPTION: 本站点的工作状态：网络处于倒向连接
// -----------------------------------------------------------------------------
#define HOST_REVERSE_LINK                                      0X30

// -----------------------------------------------------------------------------
// DESCRIPTION: 本站点的工作状态：已经完成同步,进入数据传输阶段
// -----------------------------------------------------------------------------
#define HOST_SYNCHRONIZED                                      0X40

// -----------------------------------------------------------------------------
// DESCRIPTION: 本站点的工作状态：网络通讯停止阶段
// -----------------------------------------------------------------------------
#define HOST_STOPPING_NET                                      0X80

// -----------------------------------------------------------------------------
// DESCRIPTION: 本站点的工作状态：网络节点扫描阶段
// -----------------------------------------------------------------------------
#define HOST_SCAN_NETWORK                                      0X90

// -----------------------------------------------------------------------------
// DESCRIPTION: 本站点的工作状态：参数配置模式
// -----------------------------------------------------------------------------
#define HOST_COMMAND_MODE                                      0XA0

// -----------------------------------------------------------------------------
// DESCRIPTION: 本站点的工作状态：初始化低功耗运行模式
// -----------------------------------------------------------------------------
#define HOST_INIT_LP_MODE                                      0XE0

// -----------------------------------------------------------------------------
// DESCRIPTION: 本站点的工作状态：低功耗运行模式
// -----------------------------------------------------------------------------
#define HOST_LOWPOWER_RUN                                      0XF0

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#endif