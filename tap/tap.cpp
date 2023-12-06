#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <functional>
#include <ios>
#include <iostream>
#include <cstdint>
#include <iterator>
#include <ostream>
#include <thread>
#include <unistd.h>
#include <fcntl.h>
#include <linux/if.h>
#include <linux/if_tun.h>
#include <sys/ioctl.h>
#include <span>
#include <vector>
#include <utility>
#include <arpa/inet.h>
#include <iomanip>
#include <asm/termbits.h>


class Tap {
    public:
        static constexpr size_t MTU = 1500;
        static constexpr char INTRON[] {'U', 'N', 0, 1, 2, 3, 4, 5};
        static constexpr char ssid[] = "esptest";
        static constexpr char pass[] = "lwesp8266";
        static constexpr size_t MAC_LEN {6};
        static constexpr char INTERFACE[] = "tap0";
        static constexpr char DEFAULT_TTY[] = "/dev/ttyACM2";
        // static constexpr speed_t BAUDRATE = 4705882; // Optimal for 40Mhz ESP8266 crystal
        // static constexpr speed_t BAUDRATE = 4588235; // Optimal for 26Mhz ESP8266 crystal
        static constexpr speed_t BAUDRATE = 4647059; // Compromise
        // static constexpr speed_t BAUDRATE = 115200 * 40;
        // static constexpr speed_t BAUDRATE = 1000000; // Safe
        // static constexpr speed_t BAUDRATE = 115200; // Ultrasafe

        void run(const char* tty_device) {
            std::cout << "Starting up" << std::endl;

            // Open serial
            tty = open(tty_device, O_RDWR );
            struct termios2 tio;
            memset(&tio, 0, sizeof(tio));
            tio.c_cflag = CBAUDEX | CS8 | CREAD | CLOCAL | BOTHER;
            tio.c_ispeed = BAUDRATE;
            tio.c_ospeed = BAUDRATE;
            if (ioctl(tty, TCSETS2, &tio) < 0) {
                std::cout << "TTY: Failed to set baudrate" << std::endl;
            }

            // Open tap
            tap = open("/dev/net/tun", O_RDWR);
            struct ifreq ifr;
            memset(&ifr, 0, sizeof(ifr));
            ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
            strcpy(ifr.ifr_name, INTERFACE);
            ioctl(tap, TUNSETIFF, &ifr);
            ioctl(tap, TUNSETOWNER, getuid());

            // Set MTU
            memset(&ifr, 0, sizeof(ifr));
            strcpy(ifr.ifr_name, INTERFACE);
            ifr.ifr_addr.sa_family = AF_INET;
            ifr.ifr_mtu = MTU;
            int sockfd = socket(AF_INET,SOCK_DGRAM,0);
            ioctl(sockfd, SIOCSIFMTU, &ifr);
            close(sockfd);

            // Start processing
            std::thread rx_thread(&Tap::rx, this);
            std::thread tx_thread(&Tap::tx, this);
            std::thread status_thread(&Tap::status, this);

            send_wifi_client();

            rx_thread.join();
            tx_thread.join();

            std::cout << "Shutting down" << std::endl;
            close(tty);
            close(tap); 
        }
    
    private:
        int tty;
        int tap;
        bool link_up;

        enum class Msg: uint8_t {
            DEVINFO_V2 = 0,
            CLIENTCONFIG_V2 = 6,
            PACKET_V2 = 7,
        };

        template<typename T>
        T get_tty() {
            T value;                
            read_tty(std::span<uint8_t>(reinterpret_cast<uint8_t*>(&value), sizeof(value)));
            return value;
        }

        ssize_t read_tty(std::span<uint8_t> buffer) {
            size_t pos = 0;
            while (pos < buffer.size()) {
                size_t avaialble_bytes;
                ioctl(tty, FIONREAD, &avaialble_bytes);
                if (avaialble_bytes == 0) {
                    std::cout << "TTY: POSSIBLE OVERFLOW" << std::endl;
                }
                ssize_t r = read(tty, buffer.data() + pos, buffer.size() - pos);
                if (r < 0) {
                    std::cout << "TTY: Read error" << std::endl;
                    return r;
                }
                pos += r;
            }
            return pos;
        }

        void wait_for_intron() {
            // std::cout << "TTY: Waiting for intron" << std::endl;
            
            size_t pos = 0;
            bool failed = false;
            while (pos < sizeof(INTRON)) {
                char c = get_tty<char>();
                if (c == INTRON[pos]){
                    pos++;
                    // # print(f"TAP: INTRON: pos: {pos}, byte: {bytes([c])}")
                    // #print("I", end="", flush=True)
                } else {
                    // # print(f"{safe(bytes([c]))}", end="")
                    pos = 0;
                    failed = true;
                    // #print("X", end="", flush=True)
                }
            }
            if (failed) {
                std::cout << "TTY: intron found after failure" << std::endl;
            } else {
                // std::cout << "TTY: Intron found" << std::endl;
            }
        }

        void send_message(Msg type, uint8_t data, std::span<uint8_t> payload) {
            static std::mutex send_lock;

            std::lock_guard<std::mutex> lock(send_lock);

            std::vector<uint8_t> message;

            std::copy(INTRON, INTRON + sizeof(INTRON), std::back_inserter(message));
            message.push_back(std::to_underlying(type));
            message.push_back(data);
            uint16_t size = htons(payload.size());
            std::copy(reinterpret_cast<uint8_t*>(&size), reinterpret_cast<uint8_t*>(&size) + sizeof(size), std::back_inserter(message));
            std::copy(payload.begin(), payload.end(), std::back_inserter(message));

            write(tty, message.data(), message.size());
            // std::cout << "+";

            // for(int i = 0; i < message.size(); i++) {
            //     std::cout <<  std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(message[i]) << " ";
            // }
            // std::cout << std::dec << std::endl;
            // std::cout << "TTY: " << message.size() << " B message sent" << std::endl;
        }

        void send_wifi_client() {
            std::cout << "TAP: Sending client config:  ssid: " << ssid << ", pass: " << pass << std::endl;

            std::vector<uint8_t> payload;
            uint8_t ssid_len = strlen(ssid);
            uint8_t pass_len = strlen(pass);
            std::copy(INTRON, INTRON + sizeof(INTRON), std::back_inserter(payload));
            payload.push_back(ssid_len);
            std::copy(ssid, ssid + ssid_len, std::back_inserter(payload));
            payload.push_back(pass_len);
            std::copy(pass, pass + pass_len, std::back_inserter(payload));

            send_message(Msg::CLIENTCONFIG_V2, 0, payload);
        }

        void recv_link(bool up) {
            if (up != link_up) {
                std::cout << "Received link: " << (up ? "up" : "down") << std::endl;
            }
            link_up = up;

            // Set link state
            struct ifreq ifr;
            memset(&ifr, 0, sizeof(ifr));
            strcpy(ifr.ifr_name, INTERFACE);
            ifr.ifr_flags = up ? IFF_UP : 0;
            ioctl(tap, SIOCSIFMTU, &ifr);

            if (!up) {
                send_wifi_client();
            }
        }

        void recv_packet() {
            std::cout << "TTY: Reading packet" << std::endl;
            bool up = get_tty<bool>();
            recv_link(up);
            uint16_t raw_size = get_tty<uint16_t>();
            uint16_t size = ntohs(raw_size);

            if(!size) {
                return;
            }

            if(size <= MTU + 18) {
                std::vector<uint8_t> buffer(size);
                // std::cout << "TTY: Reading " << size << " B packet" << std::endl;
                ssize_t r = read_tty(buffer);
                // std::cout << "TTY: Read " << r << " B packet" << std::endl;
                ssize_t w = write(tap, buffer.data(), r);
                // std::cout << "TAP: Written " << w << " B packet" << std::endl;
            } else {
                std::cout << "Invalid packet size: " << size << std::endl;
            }
        }

        void recv_devinfo() {
            uint8_t version = get_tty<uint8_t>();
            std::cout << "TAP: ESP FW version: " << static_cast<int>(version) << std::endl;

            uint16_t raw_size = get_tty<uint16_t>();
            uint16_t size = ntohs(raw_size);
            
            uint8_t mac[MAC_LEN];
            read_tty(std::span<uint8_t>(mac, MAC_LEN));
            std::cout << "MAC: ";
            for(int i = 0; i < MAC_LEN; i++) {
                std::cout << std::hex << static_cast<int>(mac[i]) << " ";
            }
            std::cout << std::dec << std::endl;

            struct ifreq ifr;
            
            // Set MAC
            memset(&ifr, 0, sizeof(ifr));
            strcpy(ifr.ifr_name, INTERFACE);
            ifr.ifr_hwaddr.sa_family = AF_UNIX;
            memcpy(ifr.ifr_hwaddr.sa_data, mac, std::min(sizeof(ifr.ifr_hwaddr.sa_data), sizeof(mac)));
            ioctl(tap, SIOCSIFHWADDR, &ifr);
        }

        void rx() {
            std::cout << "RX started" << std::endl;

            while(true) {
                wait_for_intron();

                Msg type = get_tty<Msg>();
                // std::cout << "TTY: Received message type: " << static_cast<int>(std::to_underlying(type)) << std::endl;
                // std::cout << "-";

                switch(type) {
                    case Msg::PACKET_V2:
                        recv_packet();
                        break;
                    case Msg::DEVINFO_V2:
                        recv_devinfo();
                        break;
                    default:
                        std::cout << "Unknown message type: " << static_cast<int>(std::to_underlying(type)) << std::endl;
                }
            }
        }

        void tx() {
            std::cout << "TX started" << std::endl;

            while(true) {
                std::vector<uint8_t> buffer(MTU + 18);
                ssize_t len = read(tap, buffer.data(), buffer.size());
                if(len > 0) {
                    std::cout << "TAP: " << len << " bytes read" << std::endl;
                    buffer.resize(len);
                    send_message(Msg::PACKET_V2, 0, buffer);
                }
                
            }
        }

        void status() {
            std::cout << "Status started" << std::endl;

            while(true) {
                sleep(30);
                send_message(Msg::PACKET_V2, 0, std::span<uint8_t>());
            }
        }
};

int main(int argc, char **argv) {
    Tap tap;
    tap.run(argc == 2 ? argv[1] : Tap::DEFAULT_TTY);
}