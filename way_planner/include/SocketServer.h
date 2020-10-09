/*
 * Copyright 2017-2020 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * SocketServer.h
 *
 *  Created on: Feb 13, 2017
 *      Author: user
 */

#ifndef SOCKETSERVER_H_
#define SOCKETSERVER_H_

#include <cstdio>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <pthread.h>
#include "ROSHelpers.h"

namespace WayPlannerNS
{

class HMISocketServer
{
  public:
    HMISocketServer();
    virtual ~HMISocketServer();
    int InitSocket(int port_send, int port_receive);
    void SendMSG(HMI_MSG msg);
    int GetLatestMSG(HMI_MSG& msg);

  private:
    int m_ConnPortSend;
    int m_ConnPortReceive;
    int m_Socket_send;
    int m_Socket_receive;
    pthread_mutex_t sock_mutex_send;
    pthread_t sock_thread_tid_send;
    HMI_MSG m_msg_send;
    bool m_bLatestMsg_send;
    pthread_mutex_t sock_mutex_receive;
    pthread_t sock_thread_tid_receive;
    HMI_MSG m_msg_receive;
    bool m_bLatestMsg_receive;

    bool m_bExitMainLoop;

    static void* ThreadMainSend(void* pSock);
    static void* ThreadMainReceive(void* pSock);
};

}  // namespace WayPlannerNS

#endif  // SOCKETSERVER_H_
