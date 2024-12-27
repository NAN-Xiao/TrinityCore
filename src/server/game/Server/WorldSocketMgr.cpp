/*
 * This file is part of the TrinityCore Project. See AUTHORS file for Copyright information
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "WorldSocketMgr.h"
#include "Config.h"
#include "NetworkThread.h"
#include "ScriptMgr.h"
#include "WorldSocket.h"
#include <boost/system/error_code.hpp>

static void OnSocketAccept(boost::asio::ip::tcp::socket &&sock, uint32 threadIndex)
{
    sWorldSocketMgr.OnSocketOpen(std::forward<boost::asio::ip::tcp::socket>(sock), threadIndex);
}

class WorldSocketThread : public NetworkThread<WorldSocket>
{
public:
    void SocketAdded(std::shared_ptr<WorldSocket> sock) override
    {
        sock->SetSendBufferSize(sWorldSocketMgr.GetApplicationSendBufferSize());
        sScriptMgr->OnSocketOpen(sock);
    }

    void SocketRemoved(std::shared_ptr<WorldSocket> sock) override
    {
        sScriptMgr->OnSocketClose(sock);
    }
};

WorldSocketMgr::WorldSocketMgr() : BaseSocketMgr(), _instanceAcceptor(nullptr), _socketSystemSendBufferSize(-1), _socketApplicationSendBufferSize(65536), _tcpNoDelay(true)
{
}

WorldSocketMgr::~WorldSocketMgr()
{
    ASSERT(!_instanceAcceptor, "StopNetwork must be called prior to WorldSocketMgr destruction");
}

WorldSocketMgr &WorldSocketMgr::Instance()
{
    static WorldSocketMgr instance;
    return instance;
}

bool WorldSocketMgr::StartWorldNetwork(Trinity::Asio::IoContext &ioContext, std::string const &bindIp, uint16 port, uint16 instancePort, int threadCount)
{
    /*
        TCP_NODELAY 是一个套接字（Socket）选项，用于控制 TCP 协议中的 Nagle 算法。Nagle 算法的主要目的是减少网络中小包的发送数量，
        它会将多个小的 TCP 数据包累积起来，等待足够的数据后再一次性发送，这样可以降低网络中因为大量小包传输而产生的开销。
        但是在某些场景下，如实时性要求较高的应用（像在线游戏、实时音视频通信等），这种延迟发送的方式可能会导致数据传输延迟过高，
        影响用户体验。此时可以通过设置 TCP_NODELAY 选项来禁用 Nagle 算法，使数据能够尽快发送，提高数据传输的实时性。
    */
    _tcpNoDelay = sConfigMgr->GetBoolDefault("Network.TcpNodelay", true);

    int const max_connections = TRINITY_MAX_LISTEN_CONNECTIONS;
    TC_LOG_DEBUG("misc", "Max allowed socket connections {}", max_connections);

    // -1 means use default
    //-1表示使用默认值 send消息缓冲区
    _socketSystemSendBufferSize = sConfigMgr->GetIntDefault("Network.OutKBuff", -1);

    _socketApplicationSendBufferSize = sConfigMgr->GetIntDefault("Network.OutUBuff", 65536);

    if (_socketApplicationSendBufferSize <= 0)
    {
        TC_LOG_ERROR("misc", "Network.OutUBuff is wrong in your config file");
        return false;
    }

    if (!BaseSocketMgr::StartNetwork(ioContext, bindIp, port, threadCount))
        return false;

    AsyncAcceptor *instanceAcceptor = nullptr;
    try
    {
        instanceAcceptor = new AsyncAcceptor(ioContext, bindIp, instancePort);
    }
    catch (boost::system::system_error const &err)
    {
        TC_LOG_ERROR("network", "Exception caught in WorldSocketMgr::StartNetwork ({}:{}): {}", bindIp, port, err.what());
        return false;
    }

    if (!instanceAcceptor->Bind())
    {
        TC_LOG_ERROR("network", "StartNetwork failed to bind instance socket acceptor");
        delete instanceAcceptor;
        return false;
    }

    _instanceAcceptor = instanceAcceptor;

    _instanceAcceptor->SetSocketFactory([this]()
                                        { return GetSocketForAccept(); });

    _acceptor->AsyncAcceptWithCallback<&OnSocketAccept>();
    _instanceAcceptor->AsyncAcceptWithCallback<&OnSocketAccept>();

    sScriptMgr->OnNetworkStart();
    return true;
}

void WorldSocketMgr::StopNetwork()
{
    if (_instanceAcceptor)
        _instanceAcceptor->Close();

    BaseSocketMgr::StopNetwork();

    delete _instanceAcceptor;
    _instanceAcceptor = nullptr;

    sScriptMgr->OnNetworkStop();
}

void WorldSocketMgr::OnSocketOpen(boost::asio::ip::tcp::socket &&sock, uint32 threadIndex)
{
    // set some options here
    if (_socketSystemSendBufferSize >= 0)
    {
        boost::system::error_code err;
        sock.set_option(boost::asio::socket_base::send_buffer_size(_socketSystemSendBufferSize), err);
        if (err && err != boost::system::errc::not_supported)
        {
            TC_LOG_ERROR("misc", "WorldSocketMgr::OnSocketOpen sock.set_option(boost::asio::socket_base::send_buffer_size) err = {}", err.message());
            return;
        }
    }

    // Set TCP_NODELAY.
    if (_tcpNoDelay)
    {
        boost::system::error_code err;
        sock.set_option(boost::asio::ip::tcp::no_delay(true), err);
        if (err)
        {
            TC_LOG_ERROR("misc", "WorldSocketMgr::OnSocketOpen sock.set_option(boost::asio::ip::tcp::no_delay) err = {}", err.message());
            return;
        }
    }

    // sock->m_OutBufferSize = static_cast<size_t> (m_SockOutUBuff);

    BaseSocketMgr::OnSocketOpen(std::forward<boost::asio::ip::tcp::socket>(sock), threadIndex);
}

NetworkThread<WorldSocket> *WorldSocketMgr::CreateThreads() const
{
    return new WorldSocketThread[GetNetworkThreadCount()];
}
