#!/usr/bin/env python3
"""
协作七自由度机械臂控制SDK封装类
文件名: robotarm_sdk.py
"""
import json
import time
import socket
import zlib
import struct
import select
import threading
import logging, colorlog
import numpy as np
import yaml
import os
from typing import Dict, Any, Optional, List, Union, Tuple
from datetime import datetime
import glob
from ament_index_python.packages import get_package_share_directory

    
class RobotArmSDK:
    """机器人控制器类，封装所有通信和控制功能"""
    def __init__(self, ip: str, port: int, log_level: int = 0):
        """
        初始化控制器

        Args:
            ip(str): 机械臂IP地址
            port(int): 机械臂端口号
            log_level(int): 日志等级
                - 0: info
                - 1: debug
        
        Returns:
            None
        """
        self.ip = ip
        self.port = port
        self.socket: Optional[socket.socket] = None
        self._connect_flag = False  # 连接状态标志
        self.return_status: Dict[str, Any] = {}  # 状态存储字典
        self._receive_thread: Optional[threading.Thread] = None
        self._status_lock = threading.Lock()
        np.set_printoptions(suppress=True, precision=4)
        
        package_share_directory = get_package_share_directory('arm_control')
        self.constants_path = os.path.join(package_share_directory, 'config', 'constants.yaml')
        SDK_path = os.path.dirname(os.path.abspath(__file__))
        self.CONSTANTS = self._load_constants(self.constants_path)

        self.log_path = SDK_path+"/log"
        if log_level == 0:
            self._setup_logger(logging.INFO)
        elif log_level == 1:
            self._setup_logger(logging.DEBUG)

        if self.port == 6001 or self.port ==7000:
            self.logger.info(f'初始化TCB710 SDK, 连接ip: {self.ip}, 连接port: {self.port}')
        else:
            self.logger.error(f'连接port错误, 检查是否为 6001 或者 7000。')

    def _setup_logger(self, log_level: int):
        os.makedirs(self.log_path, exist_ok=True)

        today_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        log_file_path = os.path.join(self.log_path, f"{today_str}.log")

        # 日志颜色配置
        log_colors = {
            'DEBUG': 'cyan',
            'INFO': 'green',
            'WARNING': 'yellow',
            'ERROR': 'red',
            'CRITICAL': 'bold_red',
        }

        formatter = colorlog.ColoredFormatter(
            '%(log_color)s%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            log_colors=log_colors
        )

        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)

        file_handler = logging.FileHandler(log_file_path, encoding='utf-8')
        file_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(file_formatter)

        self.logger = logging.getLogger('RobotArmSDK')
        self.logger.setLevel(log_level)

        if not self.logger.handlers:
            self.logger.addHandler(console_handler)
            self.logger.addHandler(file_handler)
        else:
            self.logger.propagate = False

        # 删除多余日志文件（保留最近 50 个）
        log_files = sorted(glob.glob(os.path.join(self.log_path, "*.log")), key=os.path.getmtime)
        if len(log_files) > 50:
            for old_file in log_files[:-50]:
                try:
                    os.remove(old_file)
                except Exception as e:
                    self.logger.warning(f"删除旧日志文件失败: {old_file}, 错误: {e}")

    def __enter__(self):
        """支持with上下文管理器"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """退出时自动断开连接"""
        self.disconnect()

    def connect(self):
        """建立控制器连接"""
        try:
            self.socket = socket.socket()
            self.socket.connect((self.ip, self.port))
            self._connect_flag = False
            self._start_receive_thread()
            self.logger.info(f"成功连接到 {self.ip}:{self.port}")
            return True
        except socket.error as e:
            self.logger.error(f"连接失败: {e}")
            return False

    def disconnect(self):
        """断开连接并清理资源"""
        self._connect_flag = True

        try:
            if self.socket:
                self.socket.close()
        except Exception as e:
            self.logger.error(f"关闭socket时出错: {e}")
            return False
        
        if self._receive_thread and self._receive_thread.is_alive():
            self._receive_thread.join(timeout=2)
            if self._receive_thread.is_alive():
                self.logger.warning("接收线程未能在超时时间内停止")
                return False
            
        self.logger.info("连接已断开")
        return True

    def reconnect(self, retries=3, delay=1):
        """自动重连机制"""
        for attempt in range(retries):
            try:
                self.disconnect()
                time.sleep(3)
                self.connect()
                self.logger.info(f"第{attempt+1}次重连成功")
                return True
            except Exception as e:
                self.logger.warning(f"重连失败: {e}")
                time.sleep(delay)
        return False

    def _start_receive_thread(self):
        """启动数据接收线程"""
        if self._receive_thread and self._receive_thread.is_alive():
            self.logger.info("接收线程已启动")
            return  
        self._receive_thread = threading.Thread(
            target=self._receive_loop, 
            daemon=True
        )
        self._receive_thread.start()

    def _decode_robot_data(self, raw_data: bytes):
        """
        解码包含多个 JSON 数据包的原始字节流。
        
        Args:
            raw_data (bytes): 包含多个 JSON 字符串的原始数据，每个可能带有前缀和换行符
        
        Returns:
            list[dict]: 分割成功的 JSON 字符串的原始数据的字典列表
            int: 控制器返回的命令字
        """
        decoded_messages = []
        cmd_word = f"{raw_data[4]:02X}{raw_data[5]:02X}"

        parts = raw_data.split(b'\n')

        for part in parts:
            start_index = part.find(b'{')
            if start_index == -1:
                continue  # 非 JSON 数据，跳过
            try:
                json_bytes = part[start_index:]
                decoded_messages.append(json_bytes)
            except (UnicodeDecodeError, json.JSONDecodeError) as e:
                self.logger.warning(f"解码失败，内容片段略过: {e}")
                continue

        return decoded_messages, cmd_word
    
    def _receive_loop(self):
        """
        接收数据的线程循环。

        持续监听 socket 连接的可读状态，接收来自控制器的字节流数据。
        若检测到有效数据则调用 `_parse_status()` 进行状态解析。
        线程运行条件由 `_connect_flag` 控制，当标志为 True 或 socket 无效时退出。
        """
        while True:
            if self._connect_flag or not self.socket:
                break
            try:
                while not self._connect_flag and self.socket:
                    try:
                        readable, _, _ = select.select([self.socket], [], [], 1)
                        if readable:
                            data = self.socket.recv(2048)
                            if data:
                                result, cmd_word = self._decode_robot_data(data)
                                for i in range(0, len(result)):
                                    self._parse_status(result[i], cmd_word)
                    except (OSError, ConnectionAbortedError):
                        pass 
            except Exception as e:
                # self.logger.error(f"接收线程异常: {e}")
                pass 

    def _crc(self, data_to: bytes, command: bytes, data_segment: str):
        """
        构造 CRC 校验的数据帧。

        根据协议规范，将帧头、长度、命令字节和数据段组合并附加 CRC32 校验值，
        构造完整的可发送数据帧(byte 格式)。

        Args:
            data_to (bytes): 帧头字节。
            command (bytes): 命令字节。
            data_segment (str): 需要附加的 JSON 数据内容，使用 utf-8 编码。

        Returns:
            bytes: 含 CRC 校验的完整数据帧，可直接通过 socket 发送。
        """
        length_bytes = struct.pack('>H', len(data_segment))
        crc32 = zlib.crc32(length_bytes + command + data_segment)
        crc_bytes = struct.pack('>I', crc32)
        return data_to + length_bytes + command + data_segment + crc_bytes

    def _send_command(self, cmd_word: bytes, cmd_data: str = None, keynames: List[str]=None):
        """
        发送 JSON 命令

        Args:
            cmd_word (bytes): 命令字节，用于标识具体的操作指令。
            cmd_data (str): JSON 格式的命令数据字符串，通常包含具体指令参数。

        Raises:
            ConnectionError: 若 socket 未建立有效连接。
        """
        if not self.socket:
            raise ConnectionError("未建立有效连接")

        data_to_send = self._crc(self.CONSTANTS['FRAME_HEADER'], cmd_word, json.dumps(cmd_data).encode("GBK"))
        self.logger.debug(f'发送给控制器的数据为: {data_to_send}')
        try:
            self.socket.send(data_to_send)
            # print(self.return_status)
        except (BrokenPipeError, ConnectionResetError) as e:
            self.logger.error(f"命令发送失败: {e}，尝试重连")
            if self.reconnect():
                try:
                    self.socket.send(data_to_send)  # 重连后重新发送
                    self.logger.info("命令在重连后成功发送")
                except Exception as e2:
                    self.logger.error(f"重连后发送失败: {e2}")
            else:
                self.logger.error("重连失败，命令丢失")
                self.reconnect()
                return 
        time.sleep(0.005)  
        if keynames == None:
            return
        else:
            # 等待所有需要的状态返回
            missing_keys = [key for key in keynames if key not in self.return_status]
            while missing_keys:
                self.logger.debug(f"等待状态键: {missing_keys}")
                time.sleep(0.005)
                missing_keys = [key for key in keynames if key not in self.return_status]

    def _parse_status(self, data: bytes, cmd_word):
        """解析状态数据"""
        with self._status_lock:
            json_str = data.decode('utf-8').strip()
            try:
                parsed = json.loads(json_str)
                for key, value in parsed.items():
                    status_key = cmd_word + key

                    if status_key == "5534jobfilelist":
                        # '5534jobfilelist' 数据，追加新的数据
                        if status_key in self.return_status:
                            self.return_status[status_key].extend(value)  # 添加新数据到已有列表中
                        else:
                            self.return_status[status_key] = value  # 如果没有，直接赋值
                    elif status_key == "5534listnum":
                        # '5534listnum'，进行累加
                        if status_key in self.return_status:
                            self.return_status[status_key] += value 
                        else:
                            self.return_status[status_key] = value  
                    elif status_key == "5073servo":
                        # '5073servo' 数据，追加新的数据
                        if status_key in self.return_status:
                            self.return_status[status_key].update(value) # 添加新数据到已有字典中
                        else:
                            self.return_status[status_key] = value 
                    else:
                        # 其他的字段按正常方式处理
                        self.return_status[status_key] = value

                self.logger.debug(f"状态更新: {self.return_status}\n") # 测试的时候使用

            except json.JSONDecodeError:
                return
                self.logger.error("JSON解析失败")

    def _load_constants(self, file_path):
        with open(file_path, "r") as f:
            data = yaml.safe_load(f)

        def convert_hex_to_bytes(obj):
            if isinstance(obj, dict):
                return {k: convert_hex_to_bytes(v) for k, v in obj.items()}
            elif isinstance(obj, str):
                return bytes.fromhex(obj)
            else:
                return obj

        return convert_hex_to_bytes(data)
    
    def _return_get(self, inquiry_key_name: str):
        """
        """
        key_value = self.return_status.get(inquiry_key_name, None)
        status_flag = 0
        while key_value == None:
            time.sleep(0.005)
            key_value = self.return_status.get(inquiry_key_name, None)
            status_flag += 1
            if status_flag >= 3:
                break
        return key_value
    
    def fault_reset(self, robot: int):
        """
        清除伺服错误

        Args:
            robot: 机器人号码, 默认为 1
        
        Returns:
            bool: 清错状态标志
                - True:  清错成功
                - False: 清错失败 (通过示教器查看无法清除的错误类型, 告知专业人员处理。)
        """
        cmd_data = {"robot":robot}
        self._send_command(self.CONSTANTS['FAULT_RESET'], cmd_data)
        clearErrflag = self._return_get('3202clearErrflag')
        if clearErrflag == True:
            self.logger.info(f"清除伺服错误 成功")
        else:
            self.logger.info(f"清除伺服错误 失败")
        return clearErrflag
    
    def reboot_controller(self):
        """
        重启控制器

        Args:
            None

        Returns:
            None
        """
        self._send_command(self.CONSTANTS['REBOOT_CONTROLLER'])
        self.logger.info(f"重启控制器")
    
    def controller_init_finish_inquire(self):
        """
        控制器初始化是否完成

        Args:
            None
        
        Returns:
            bool:控制器初始化状态
                - True: 控制器初始化完成
                - False: 控制器初始化未完成
        """
        self._send_command(self.CONSTANTS['CONTROLLER_INIT_FINISH'])
        time.sleep(0.5)
        finishinit = self._return_get('4306finishinit')
        if finishinit == True:
            self.logger.info(f'控制器初始化 完成')
        else:
            self.logger.info(f'控制器初始化 未完成')
        return finishinit

    def controller_ip_inquire(self):
        """
        控制器ip查询

        Args:
            None
        
        Returns:
            list:返回的是当前控制器的网口的ip、dns、gateway以及网口名称
        """
        self._send_command(self.CONSTANTS['CONTROLLER_IP']['INQUIRE'])
        time.sleep(0.1)
        num = self._return_get('4303num')
        self.logger.info(f'网络ip数量: {num}')
        for number in range(0, num):
            address = self.return_status.get('4303network')[number]['address']
            eth = self.return_status.get('4303network')[number]['name']
            self.logger.info(f'网口 {eth}: {address}')
        return self._return_get('4303network')
        
    def controller_ip_set(self, name: str, address: str, gateway: str= "", dns: str= ""):
        """
        控制器ip设置

        Args:
            name(str): 网口名称
            address(str): ip地址
            gateway(str): 网关
            dns(str): DNS
        
        Returns:
            None
        """
        cmd_data = {
            "name": name,
            "address": address,
            "gateway": gateway,
            "dns": dns
            }
        self._send_command(self.CONSTANTS['CONTROLLER_IP']['SET'], cmd_data)
        time.sleep(0.5)
        self.logger.info(f'修改控制器网口 {name} 的ip为 {address}, 修改后控制器自动重启(ip修改前后相同不重启)')

    def control_cycle_set(self, controlCycle: int):
        """
        设置控制器通讯周期

        Args:
            controlCycle: 参数为 1、2、4、8 毫秒(ms), 控制器重启生效

        Returns:
            None  
        """
        cmd_data = {"controlCycle":controlCycle}
        self._send_command(self.CONSTANTS['CONTROL_CYCLE']['SET'], cmd_data)
        time.sleep(0.5)
        self.logger.info(f"机器人通讯周期设置为: {controlCycle}")
        self.logger.info(f"机器人通讯周期设置后, 控制器重启生效")

    def control_cycle_inquire(self):
        """
        查询控制器通讯周期

        Args:
            None
        
        Returns:
            int: 控制器通讯周期, 单位 毫秒(ms)
        """
        self._send_command(self.CONSTANTS['CONTROL_CYCLE']['INQUIRE'])
        controlCycle = self._return_get('2E09controlCycle')
        self.logger.info(f"机器人通讯周期为 {controlCycle} ms")
        return controlCycle

    def deadman_status_set(self, deadman: int):
        """
        设置伺服上下电状态

        Args:
            deadman: 上下电状态
                - 0: 机器人下电
                - 1: 机器人上电
        
        Returns:
            int: 上下电状态
                - 0: 机器人下电
                - 1: 机器人上电
        """
        cmd_data = {"deadman":deadman}
        self._send_command(self.CONSTANTS['DEADMAN_STATUS']['SET'], cmd_data)
        time.sleep(0.5)
        deadman_map = {
            0: "下电状态",
            1: "上电状态",
        }
        self.logger.info(f"设置上下电状态: {deadman_map.get(deadman, '未知状态')}")
        return deadman

    def deadman_status_inquire(self):
        """
        查询伺服上下电状态

        Args:
            None
        
        Returns:
            int: 上下电状态
                - 0: 机器人下电
                - 1: 机器人上电
        """
        self._send_command(self.CONSTANTS['DEADMAN_STATUS']['INQUIRE'])
        deadman_return = self._return_get('2303deadman')
        deadman_map = {
            0: "下电状态",
            1: "上电状态",
        }
        self.logger.info(f"查询上下电状态: {deadman_map.get(deadman_return, '未知状态')}")
        return deadman_return

    def servo_status_set(self, robot: int, status: int):
        """
        设置伺服状态

        Args:
            robot(int): 机器人号码
            status(int): 需要设置的伺服状态
                - 0: 停⽌
                - 1: 就绪
                - 2: 错误
                - 3: 运⾏
            
        Returns:
            int: 设置后的伺服状态
                - 0: 停⽌
                - 1: 就绪
                - 2: 错误
                - 3: 运⾏
        """
        cmd_data = {
            "robot":robot,
            "status":status
            }
        self._send_command(self.CONSTANTS['SERVO_COMMANDS']['SET'], cmd_data)
        status_map = {
            0: "伺服 停止",
            1: "伺服 就绪",
            2: "伺服 错误",
            3: "伺服 运行"
        }
        self.logger.info(f"设置伺服状态: {status_map.get(status, '未知状态')}")
        return status

    def servo_status_inquire(self, robot: int):
        """
        查询伺服状态

        Args:
            robot(int): 机器人号码
        
        Returns:
            int: 机器人当前伺服状态
                - 0: 停⽌
                - 1: 就绪
                - 2: 错误
                - 3: 运⾏
        """
        cmd_data = {"robot":robot}
        self._send_command(self.CONSTANTS['SERVO_COMMANDS']['INQUIRE'], cmd_data)
        # time.sleep(0.5)
        status_return = self._return_get('2003status')
        status_map = {
            0: "伺服 停止",
            1: "伺服 就绪",
            2: "伺服 错误",
            3: "伺服 运行"
        }
        self.logger.info(f"查询伺服状态: {status_map.get(status_return, '未知状态')}")
        return status_return
    
    def servo_connect_inquire(self):
        """
        查询伺服连接状态

        Args: 
            None
        
        Returns:
            int: 伺服连接状态
                - 0: 真实伺服
                - 1: 虚拟伺服
                - 2: 无伺服
        """
        self._send_command(self.CONSTANTS['SERVO_CONNECT_INQUIRE'])
        servoType = self._return_get('5043servoConnect')
        if servoType == 0:
            self.logger.info(f"伺服连接状态: 真实伺服")
        elif servoType ==1:
            self.logger.info(f"伺服连接状态: 虚拟伺服")
        elif servoType == 2:
            self.logger.warning(f"伺服连接状态: 无伺服")
        return servoType

    def servo_inside_parm_inqure(self, robot: int, servoNum: int):
        """查询伺服参数（确保返回值不为 None）"""
        cmd_data = {
            "robot": robot,
            "servoNum": servoNum
        }
        self._send_command(self.CONSTANTS['SERVO_INSIDE_PARM']['INQUIRE'], cmd_data)
        time.sleep(2)
        # time.sleep(1)

        def extract_values():
            """从 return_status 中提取值（添加默认值）"""
            try:
                servo_data = self.return_status.get('5073servo', {})
                return (
                    servo_data.get('编码器错误码', {}).get('value'),
                    servo_data.get('编码器状态寄存器1', {}).get('value'),
                    servo_data.get('编码器状态寄存器2', {}).get('value'),
                    servo_data.get('编码器单圈值', {}).get('value'),
                    servo_data.get('抱闸手动控制', {}).get('value'),
                )
            except Exception as e:
                self.logger.warning(f'提取伺服参数异常: {e}')
                return None, None, None, None, None
        
        error_code, encoder_status_register1, encoder_status_register2, encoder_single_turn_value, holding_brake_status = extract_values()
        
        # 重试一次（保持原有逻辑）
        if error_code is None or encoder_status_register1 is None or encoder_status_register2 is None or encoder_single_turn_value is None or holding_brake_status is None:
            self.logger.warning(f"伺服 {servoNum} 部分参数为空，等待 2 秒后重试一次查询")
            time.sleep(2)
            error_code, encoder_status_register1, encoder_status_register2, encoder_single_turn_value, holding_brake_status = extract_values()
        
        # 最终确保所有返回值不为 None（使用默认值）
        if error_code is None:
            error_code = -1  # 用 -1 表示查询失败
        if encoder_status_register1 is None:
            encoder_status_register1 = 0
        if encoder_status_register2 is None:
            encoder_status_register2 = 0
        if encoder_single_turn_value is None:
            encoder_single_turn_value = 0
        if holding_brake_status is None:
            holding_brake_status = 0
        
        self.logger.info(f'伺服 {servoNum} 编码器错误码: {hex(error_code)}')
        self.logger.info(f'伺服 {servoNum} 编码器单圈值: {encoder_single_turn_value}')
        self.logger.info(f'伺服 {servoNum} 编码器状态寄存器1: {encoder_status_register1}')
        self.logger.info(f'伺服 {servoNum} 编码器状态寄存器2: {encoder_status_register2}')
        self.logger.info(f'伺服 {servoNum} 抱闸手动控制状态: {holding_brake_status}')

        return error_code, encoder_status_register1, encoder_status_register2, encoder_single_turn_value, holding_brake_status

    # def servo_inside_parm_inqure(self, robot: int, servoNum: int):
    #     """
    #     查询伺服参数

    #     Args:
    #         robot(int): 机器人号码
    #         servoNum(int): 伺服号
        
    #     Returns:
    #         Tuple[int, int, int, int, int]:
    #             - int: 编码器错误码(这里返回的十进制, 需要自己转为十六进制查询)
    #             - int: 编码器状态寄存器1
    #             - int: 编码器状态寄存器2
    #             - int: 编码器单圈值
    #             - int: 抱闸手动控制状态
    #                 - 0: 抱闸关闭
    #                 - 1: 抱闸打开
    #     """
    #     cmd_data = {
    #         "robot":robot,
    #         "servoNum":servoNum
    #     }
    #     self._send_command(self.CONSTANTS['SERVO_INSIDE_PARM']['INQUIRE'], cmd_data)
    #     time.sleep(2)

    #     def extract_values():
    #         """从 return_status 中提取值"""
    #         try:
    #             servo_data = self.return_status.get('5073servo', {})
    #             return (
    #                 servo_data.get('编码器错误码', {}).get('value'),
    #                 servo_data.get('编码器状态寄存器1', {}).get('value'),
    #                 servo_data.get('编码器状态寄存器2', {}).get('value'),
    #                 servo_data.get('编码器单圈值', {}).get('value'),
    #                 servo_data.get('抱闸手动控制', {}).get('value'),
    #             )
    #         except Exception as e:
    #             self.logger.warning(f'提取伺服参数异常: {e}')
    #             return None, None, None, None, None
        
    #     error_code, encoder_status_register1, encoder_status_register2, encoder_single_turn_value, holding_brake_status = extract_values()
        
    #     if error_code is None or encoder_status_register1 is None or encoder_status_register2 is None or encoder_single_turn_value is None or holding_brake_status is None:
    #         self.logger.warning("部分伺服参数为空，等待 2 秒后重试一次查询")
    #         time.sleep(2)
    #         error_code, encoder_status_register1, encoder_status_register2, encoder_single_turn_value, holding_brake_status = extract_values()

    #     self.logger.info(f'伺服 {servoNum} 编码器错误码: {hex(error_code)}')
    #     self.logger.info(f'伺服 {servoNum} 编码器单圈值: {encoder_single_turn_value}')
    #     self.logger.info(f'伺服 {servoNum} 编码器状态寄存器1: {encoder_status_register1}')
    #     self.logger.info(f'伺服 {servoNum} 编码器状态寄存器2: {encoder_status_register2}')
    #     self.logger.info(f'伺服 {servoNum} 抱闸手动控制状态: {holding_brake_status}')

    #     return error_code, encoder_status_register1, encoder_status_register2, encoder_single_turn_value, holding_brake_status

    def servo_inside_parm_set(self, robot: int, servoNum: int, key_name: str, key_value: int, temporary_save: int=1):
        """
        伺服参数设置

        Args:
            robot(int): 机器人号码
            servoNum(int): 伺服号
            key_name(str): 伺服参数名
            key_value(int): 伺服参数值
            temporary_save(int):
                - 0: 修改
                - 1: 临时存储(上下电之后恢复原来值)

        Returns:
            None
        """
        VALID_SERVO_PARAMS = ["6041", "6072", "60E0", "60E1",
                              "位置环比例增益1", "初始化指令", "抱闸关闭延时", "抱闸启动延时", "抱闸手动控制",
                              "母线电压值", "电压峰值", "电压最低值", "电机硬件版本", "电机编码", "电机软件版本", 
                              "电流环比例增益", "电流环积分时间常数", "编码器单圈值", "编码器命令", "编码器多圈值",
                              "编码器状态寄存器1", "编码器状态寄存器2", "编码器状态寄存器3", "编码器错误码", "警告状态",
                              "输入侧温度值", "速度环增益", "速度环积分时间常数"] 

        if key_name not in VALID_SERVO_PARAMS:
            # raise ValueError(f"无效的参数名: {key_name}，有效参数为: {VALID_SERVO_PARAMS}")
            self.logger.warning(f"无效的参数名: {key_name}，有效参数为: {VALID_SERVO_PARAMS}")
        else:
            cmd_data = {
                "robot": robot,
                "servo": {key_name:{"value":key_value}},
                "servoNum": servoNum,
                "temporary_save": temporary_save 
            }
            self._send_command(self.CONSTANTS['SERVO_INSIDE_PARM']['SET'], cmd_data)
            self.logger.info(f'修改伺服 {servoNum} 的伺服参数 "{key_name}" 的值为 {key_value}')
    
    def slavetype_list_respond(self):
        """
        查询从站列表

        Args:
            None

        Returns:
            tuple[list, list, list, list]:
                - IONum (list): 从站的 I/O 编号
                - servoNum (list): 从站的伺服编号
                - slaveType (list): 从站的伺服型号（中文）
                - slaveTypeEnglish (list): 从站的伺服型号（英文）
        """
        self._send_command(self.CONSTANTS['SLAVETYPE_LIST_INQUIRE'])
        IONum = self._return_get('2E0FIONum')
        servoNum = self._return_get('2E0FservoNum')
        slaveType = self._return_get('2E0FslaveType')
        slaveTypeEnglish = self._return_get('2E0FslaveTypeEnglish')
        self.logger.info(f'IO编号: {IONum}')
        self.logger.info(f'伺服编号: {servoNum}')
        self.logger.info(f'伺服型号 中文: {slaveType}')
        self.logger.info(f'伺服型号 英文: {slaveTypeEnglish}')
        return IONum, servoNum, slaveType, slaveTypeEnglish
        
    def operation_mode_set(self, mode: int):
        """
        设置操作模式状态

        Args: 
            mode(int):
                - 0: ⽰教模式(Teach)
                - 1: 远程模式(Circle)
                - 2: 运⾏模式(Repeat)

        Returns:
            None(这里没有要返回值，因为控制器返回有时候比较慢，会卡一下)
        """
        cmd_data = {"mode": mode}
        self._send_command(self.CONSTANTS['OPERATION_MODE']['SET'], cmd_data)
        time.sleep(0.5)
        mode_map = {
            0: "⽰教模式(Teach)",
            1: "远程模式(Circle)",
            2: "运⾏模式(Repeat)",
        }
        self.logger.info(f"设置操作模式: {mode_map.get(mode, '未知模式')}")
        # return self.return_status.get('2103mode', 0)

    def operation_mode_inquire(self):
        """
        查询操作模式状态

        Args:
            None
        
        Returns:
            int: 操作模式状态
                - 0: ⽰教模式(Teach)
                - 1: 远程模式(Circle)
                - 2: 运⾏模式(Repeat)
        """
        self._send_command(self.CONSTANTS['OPERATION_MODE']['INQUIRE'])
        mode = self._return_get('2103mode')
        # mode_names = ["示教模式", "远程模式", "运行模式"]
        # self.logger.info(f"当前模式: {mode_names[mode] if mode <3 else '未知模式'}")
        mode_map = {
            0: "⽰教模式(Teach)",
            1: "远程模式(Circle)",
            2: "运⾏模式(Repeat)",
        }
        self.logger.info(f"查询操作模式: {mode_map.get(mode, '未知模式')}")      
        return mode

    def jointparameter_set(self, AxisNum: int, PosSWLimit: float, NegSWLimit: float, Direction:int=1):
        """
        设置关节参数

        Args:
            AxisNum(int): 关节轴数
            PosSWLimit(float):关节正限位
            NegSWLimit(float):关节反限位
            Direction(int):模型方向, 四号关节设置前查询一下是否与其他关节相反
                - 1: 正向
                - -1: 反向
        
        Returns:
            None
        """
        if AxisNum == 4:
            Direction = -1
        cmd_data = {
            "Joint":{
                "AxisDirection":1,
                "AxisNum":AxisNum,
                "BackLash":0.0,
                "DeRatedVel":-180.0,
                "Direction":Direction,
                "EncoderResolution":19,
                "MaxAcc":1.0,
                "MaxDeRotSpeed":-1.0,
                "MaxDecel":-1.0,
                "MaxJerkAcc":1.0,
                "MaxJerkDec":-1.0,
                "MaxRotSpeed":1.0,
                "NegSWLimit":NegSWLimit,
                "PosSWLimit":PosSWLimit,
                "RatedDeRotSpeed":-3000.0,
                "RatedRotSpeed":3000,
                "RatedVel":180,
                "ReducRatio":100,
                "reduce_ratio_enable":True
                }
            }
        self._send_command(self.CONSTANTS['JOINTPARAMETER']['SET'], cmd_data)
        self.logger.info(f'关节 {AxisNum} 参数已经设置, 关节正限位: {PosSWLimit} °, 关节反限位: {NegSWLimit} °\n')

    def jointparameter_inquery(self, AxisNum: int):
        """
        查询关节参数

        Args:
            AxisNum(int): 1 - 7 分别代表关节 1 - 7

        Returns:
            tuple[float, float, float, float, float, float]:
                - PosSWLimit(float):      关节正限位
                - NegSWLimit(float):      关节反限位
                - RatedRotSpeed(float):   关节额定正转速
                - RatedDeRotSpeed(float): 关节额定反转速
                - RatedVel(float):        关节额定正速度
                - DeRatedVel(float):      关节额定反速度
        """
        cmd_data = {"AxisNum": AxisNum}
        self._send_command(self.CONSTANTS['JOINTPARAMETER']['INQUIRE'], cmd_data)
        time.sleep(0.1)
        AxisDirection       = self.return_status.get('3B03Joint')['AxisDirection']
        AxisNum             = self.return_status.get('3B03Joint')['AxisNum']
        BackLash            = self.return_status.get('3B03Joint')['BackLash']
        DeRatedVel          = self.return_status.get('3B03Joint')['DeRatedVel']
        Direction           = self.return_status.get('3B03Joint')['Direction']
        EncoderResolution   = self.return_status.get('3B03Joint')['EncoderResolution']
        MaxAcc              = self.return_status.get('3B03Joint')['MaxAcc']
        MaxDeRotSpeed       = self.return_status.get('3B03Joint')['MaxDeRotSpeed']
        MaxDecel            = self.return_status.get('3B03Joint')['MaxDecel']
        MaxJerkAcc          = self.return_status.get('3B03Joint')['MaxJerkAcc']
        MaxJerkDec          = self.return_status.get('3B03Joint')['MaxJerkDec']
        MaxRotSpeed         = self.return_status.get('3B03Joint')['MaxRotSpeed']
        NegSWLimit          = self.return_status.get('3B03Joint')['NegSWLimit']
        PosSWLimit          = self.return_status.get('3B03Joint')['PosSWLimit']
        RatedDeRotSpeed     = self.return_status.get('3B03Joint')['RatedDeRotSpeed']
        RatedRotSpeed       = self.return_status.get('3B03Joint')['RatedRotSpeed']        
        RatedVel            = self.return_status.get('3B03Joint')['RatedVel']
        ReducRatio          = self.return_status.get('3B03Joint')['ReducRatio']
        reduce_ratio_enable = self.return_status.get('3B03Joint')['reduce_ratio_enable']
        self.logger.info(f'查询参数关节: {AxisNum} 关节')        
        self.logger.info(f'关节正限位: {PosSWLimit} 度')
        self.logger.info(f'关节反限位: {NegSWLimit} 度')
        self.logger.info(f'减速比: {ReducRatio}')
        self.logger.info(f'编码器位数: {EncoderResolution}')  
        self.logger.info(f'关节额定正转速: {RatedRotSpeed} 转/分钟')           
        self.logger.info(f'关节额定反转速: {RatedDeRotSpeed} 转/分钟')
        self.logger.info(f'最大正转速: {MaxRotSpeed} 倍数')
        self.logger.info(f'最大反转速: {MaxDeRotSpeed} 倍数')        
        self.logger.info(f'关节额定正速度: {RatedVel} 度/秒')
        self.logger.info(f'关节额定反速度: {DeRatedVel} 度/秒')
        self.logger.info(f'最大加速度: {MaxAcc} 倍数')
        self.logger.info(f'最大减速度: {MaxDecel} 倍数')
        self.logger.info(f'最大加加速度: {MaxJerkAcc}')
        self.logger.info(f'最大减减速度: {MaxJerkDec}')
        self.logger.info(f'关节实际方向: {AxisDirection}')
        self.logger.info(f'模型方向: {Direction}')
        self.logger.info(f'齿轮反向间隙: {BackLash}')
        self.logger.info(f'编码器是否经过减速机: {reduce_ratio_enable}')
        return PosSWLimit, NegSWLimit, RatedRotSpeed, RatedDeRotSpeed, RatedVel, DeRatedVel

    def jobsend_done(self, robot: int, jobname: str, line: int, continueRun: int):
        """
        开始运行作业文件

        Args:
            robot(int): 机器人号码
            jobname(str): 作业文件名字
            line(int): 作业⽂件指令⾏数,不能为零，不能超过总⾏数
            continueRun(int): 1:继续运⾏,0:不继续运⾏
        
        Returns: 
            None
        """
        cmd_data = {
            "robot":robot,
            "jobname":jobname,
            "line":line,
            "continueRun":continueRun
            }
        self.logger.info(f"运行作业文件：{jobname}")
        # self._send_command(self.CONSTANTS['JOB_CONTROL']['JOBSEND_DONE'], cmd_data, ['2B04kind'])
        self._send_command(self.CONSTANTS['JOB_CONTROL']['JOBSEND_DONE'], cmd_data)
        time.sleep(1)
        # return self.return_status.get('2B04kind', 0)

    def stop_job_run(self, robot: int):
        """
        停止正在运行的作业文件

        Args:
            robot(int): 机器人号码    

        Returns: 
            None   
        """
        cmd_data = {"robot":robot}
        self._send_command(self.CONSTANTS['JOB_CONTROL']['STOP_JOB_RUN'], cmd_data)
        self.logger.info(f'停止正在运行的作业文件')

    def jobfile_list_inquire(self):
        """
        获取作业文件列表

        Args:
            None
        
        Returns:
            list:作业文件列表
        """
        self._send_command(self.CONSTANTS['JOBFILE_LIST_INQUIRE'])
        time.sleep(0.1)
        absolutepath = self.return_status.get('5533absolutepath')
        jobfilenum   = self.return_status.get('5533jobfilenum')
        jobfilelist  = self.return_status.get('5534jobfilelist')
        listnum      = self.return_status.get('5534listnum')
        self.logger.info(f'作业文件路径: {absolutepath}')
        self.logger.info(f'各作业文件路径下的作业文件数量: {jobfilenum}')
        self.logger.info(f'{listnum}个文件为: {jobfilelist}')
        return jobfilelist

    def speed_set(self, robot: int, speed: int):
        """
        设置全局速度

        Args:
            robot(int): 机器人号码
            speed(int): 全局速度设置   0-100
                - 0-100: 速度值, 百分比
                - 101: 0.1°微动档
                - 102: 0.01°微动档
                - 103: 0.001°微动档
        
        Returns: 
            int: 控制器返回设置后的速度值
                - 0-100: 速度值, 百分比
                - 101: 0.1°微动档
                - 102: 0.01°微动档
                - 103: 0.001°微动档
        """
        if not 0 <= speed <= 103:
            # raise ValueError("速度值必须在0-103之间")
            self.logger.warning(f'速度值必须在0-103之间')
        else:
            cmd_data = {"robot":robot,"speed":speed}
            self._send_command(self.CONSTANTS['SPEED_CONTROL']['SET'], cmd_data)
            time.sleep(0.5)
            self.logger.info(f"设置全局速度：{speed}%")
            return speed
    
    def speed_inquire(self, robot: int):
        """
        查询全局速度

        Args:
            robot(int): 机器人号码

        Returns: 
            int: 控制器返回当前的速度值
                - 0-100: 速度值, 百分比
                - 101: 0.1°微动档
                - 102: 0.01°微动档
                - 103: 0.001°微动档
        """
        cmd_data = {"robot":robot}
        self._send_command(self.CONSTANTS['SPEED_CONTROL']['INQUIRE'], cmd_data)
        speed = self._return_get('2603speed')
        self.logger.info(f"查询全局速度: {speed}%")
        return speed

    def currentpos_inquiry(self, robot: int, coord: int):
        """
        获取当前位置

        Args:
            robot(int): 机器人号码
            coord(int): 坐标模式
                - -1: 控制器当前坐标
                - 0:  关节坐标(Joint)
                - 1:  直角坐标(Cart)
                - 2:  工具坐标(Tool)
                - 3:  用户坐标(User)
            
        Returns:
            List[float]: 角度点位值
                - 关节坐标分别代表1-7关节角度值
                - 直角坐标系分别x,y,z,a,b,c。工具、用户同直角。直角坐标系下第七位参数默认为0即可。
        """
        cmd_data = {"robot":robot,"coord":coord}
        self._send_command(self.CONSTANTS['CURRENTPOS_INQUIRE'], cmd_data, ['2A03pos'])
        position = self._return_get('2A03pos')
        self.logger.debug(f"当前位置: {position}")
        return position

    def directmotion_mode_set(self, robot: int, open: bool):
        """
        开启/关闭socket直接控制运动模式
        (开启后会进去特殊的运行模式，关闭后需要手动切回示教模式)

        Args:
            robot(int): 机器人号码
            open(bool): true/false
                true:  开启socket直接控制运动模式
                false: 关闭socket直接控制运动模式

        Returns:
            bool: 开启/关闭状态
                - True:  socket直接控制运动模式开启
                - False: socket直接控制运动模式关闭
        """
        cmd_data = {"robot":robot, "open":open}
        self._send_command(self.CONSTANTS['DIRECTMOTION']['MODE_SET'], cmd_data)
        time.sleep(1)
        open_map = {
            True: "开启",
            False: "关闭"
        }
        self.logger.info(f"直接控制运动模式: {open_map.get(open, '未知状态')}")
        return open

    def directmotion_insert_instrvec(self, pos: List[float], acc: int =30, dec: int=30, pl: int=4, velocity: int=100, imovecoord: str ="RF", move_type:int =1):
        """
        发送指令队列控制机械臂移动, 采用自动点位

        Args:
            ParaACC(int): 加速度  1-100    
            ParaDEC(int): 减速度  1-100   
            ParaPL(int): 平滑系数  0-5      不需要平滑填写0
            ParaSPIN(int): 圆弧和整圆指令, 0=姿态不变 1=六轴不转 2=六轴旋转  不需要填写0
            ParaTIME(int): 提前跳出该点往下执行的时间设置  不需要填写0
            ParaV(int): 速度1-100 这个是全局速度的百分比
            m_vUnit(int): 速度单位: 0 表示 cm/s, 1 表示 mm/s, 2 表示 百分比, 注意: 关节坐标填写2, 直角等坐标填写 0 或者 1 建议填写1

            data(List[float]): [0.0,0.0,0.0,0.0,0.0,0.0,0.0,%s,%s,%s,%s,%s,%s,%s,0.0,0.0,0.0,0.0,0.0,0.0,0.0 ]
                - 第 1、2 位表示坐标 0 0 ：表示关节坐标(其中第二位角度-0 、弧度-1) 1 1: 表示直角坐标 2 1: 工具坐标 3 1: 用户坐标
                - 第 3 位 左右手 1-左 2-右 0-无左右手 默认为 0
                - 第 4, 5, 6, 7 位备用，默认为 0
                - 第 8 至 14 位保存机器人本体 坐标值(7 位)
                    - 关节坐标下，分别表示 1 到 7 轴的角度值
                    - 其他坐标下，分别表示 x,y,z,a,b,c 六个轴的坐标
                    (按顺序填写 后面无值默认为 0.0 示例中,1.0,2.0,3.0,4.0,5.0,6.0 为关节坐标值)
                - 第 15 至 19 位 保存外部轴坐标值（最大支持五个外部轴，外部轴只有关节值，不足五个外部轴后关节坐标值补零）

            imovecoord(str): 
                - "RF": 关节坐标
                - "BF": 直角坐标
                - "TF": 工具坐标
                - "UF": 用户坐标
            type(int): 
                - 1: 点到
                - 2: 直线
                - 3: 圆弧
                - 4: 整圆
        
        Returns:
            None
        """
        cmd_data = {
            "data": [
                {
                    "ParaACC":  {"data": acc, "secondvalue": 0,"value": 0,"varname": "" },
                    "ParaDEC":  {"data": dec, "secondvalue": 0,"value": 0,"varname": "" },
                    "ParaPL":   {"data": pl, "secondvalue": 0,"value": 0,"varname": "" },
                    "ParaSPIN": {"data": 0.0,"secondvalue": 0,"value": 0,"varname": "" },
                    "ParaSYNC": {"data": 0.0,"secondvalue": 0,"value": 0,"varname": "" },
                    "ParaTIME": {"data": 0.0,"secondvalue": 0,"value": 0,"varname": "" },
                    "ParaV":    {"data": velocity, "m_vUnit": 2,"secondvalue": 0,"value": 0,"varname": "" },
                    "RobotPos": {
                        "ctype": 1,
                        "data": [0.0,0.0,0.0,0.0,0.0,0.0,0.0,pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],pos[6],0.0,0.0,0.0,0.0,0.0,0.0,0.0 ],
                        "key": "",
                        "paraVarData": [
                            {"data": 0.0,"secondvalue": 0,"value": 0,"varname": "" },
                            {"data": 0.0,"secondvalue": 0,"value": 0,"varname": "" },
                            {"data": 0.0,"secondvalue": 0,"value": 0,"varname": "" },
                            {"data": 0.0,"secondvalue": 0,"value": 0,"varname": "" },
                            {"data": 0.0,"secondvalue": 0,"value": 0,"varname": "" },
                            {"data": 0.0,"secondvalue": 0,"value": 0,"varname": "" },
                            {"data": 0.0,"secondvalue": 0,"value": 0,"varname": "" },
                            {"data": pos[0], "secondvalue": 0,"value": 0,"varname": "" },
                            {"data": pos[1], "secondvalue": 0,"value": 0,"varname": "" },
                            {"data": pos[2], "secondvalue": 0,"value": 0,"varname": "" },
                            {"data": pos[3], "secondvalue": 0,"value": 0,"varname": "" },
                            {"data": pos[4], "secondvalue": 0,"value": 0,"varname": "" },
                            {"data": pos[5], "secondvalue": 0,"value": 0,"varname": "" },
                            {"data": pos[6], "secondvalue": 0,"value": 0,"varname": "" },
                            {"data": 0.0,"secondvalue": 0,"value": 0,"varname": "" },
                            {"data": 0.0,"secondvalue": 0,"value": 0,"varname": "" },
                            {"data": 0.0,"secondvalue": 0,"value": 0,"varname": "" },
                            {"data": 0.0,"secondvalue": 0,"value": 0,"varname": "" },
                            {"data": 0.0,"secondvalue": 0,"value": 0,"varname": "" },
                            {"data": 0.0,"secondvalue": 0,"value": 0,"varname": "" } ]
                            },
                    "ctype": 0,
                    "imovecoord": imovecoord,
                    "length": 0.0,
                    "logout": False,
                    "margin": 0.0,
                    "offsetAxis": 0,
                    "para": 0,
                    "polish": 0,
                    "polishAngle": 0.0,
                    "polishID": 1,
                    "posidname": "",
                    "posidtype": 0,
                    "positionId": "",
                    "radius": 0.0,
                    "side": 0.0,
                    "type": move_type,
                    "userParamInt": 0,
                    "userParamString": "",
                    "width": 0.0}],
        "robot": 1}

        self._send_command(self.CONSTANTS['DIRECTMOTION']['INSERT_INSTRVEC'], cmd_data)
        # self.logger.info(f'已发送一组队列')
        
    def directmotion_mode_suspend(self, robot: int):
        """
        暂停追加队列运行

        Args:
            robot(int): 机器人号码
        
        Returns:
            None
        """
        cmd_data = {"robot":robot}
        self._send_command(self.CONSTANTS['DIRECTMOTION']['MODE_SUSPEND'], cmd_data)
        self.logger.info(f'暂停 追加队列运行')
        time.sleep(2)

    def directmotion_mode_start(self, robot: int):
        """
        开始追加队列运行（暂停后，发送使用）

        Args:
            robot(int): 机器人号码
        
        Returns:
            None
        """
        cmd_data = {"robot":robot}
        self._send_command(self.CONSTANTS['DIRECTMOTION']['MODE_START'], cmd_data)
        self.logger.info(f'开始 追加队列运行')
        time.sleep(2)

    def directmotion_mode_stop(self, robot: int):
        """
        停止追加队列运行

        Args:
            robot(int): 机器人号码
        
        Returns:
            None
        """
        cmd_data = {"robot":robot}
        self._send_command(self.CONSTANTS['DIRECTMOTION']['MODE_STOP'], cmd_data)
        self.logger.info(f'停止 追加队列运行')
        time.sleep(2)

    def directmotion_keep_power_on(self, robot: int):
        """
        设置队列模式停止不下电
        (目前控制器暂不支持该指令)

        Args:
            robot(int): 机器人号码
        
        Returns:
            None
        """
        cmd_data = {"robot":robot}
        self._send_command(self.CONSTANTS['DIRECTMOTION']['KEEP_POWER_ON'], cmd_data)
        self.logger.info(f'设置队列模式停止不下电')
        time.sleep(2)

    def movj(self, robot: int, vel: int, coord: int, pos: List[float]):
        """
        机器人关节运动MOVJ
       
        Args:
            robot(int): 机器人号码
            vel(int): 速度百分⽐,1-100的整数
            coord(int): 坐标模式
                - 0:  关节坐标(Joint)
                - 1:  直角坐标(Cart)
                - 2:  工具坐标(Tool)
                - 3:  用户坐标(User)
            pos(List[float]):[1.1,2.2,3.3,4.4,5.5,6.6,7.7]
                - 关节坐标分别代表1-7关节角度值
                - 直角坐标系分别x,y,z,a,b,c。工具、用户同直角。直角坐标系下第七位参数默认为0即可。
        
        Returns:
            None
        """
        cmd_data = {"robot":robot, "vel":vel, "coord":coord, "pos":pos}
        self._send_command(self.CONSTANTS['ROBOT_MOVEMENT']['JOINT'], cmd_data)
        self.logger.info(f"已发送移动的关节角度(movj)\n>>>>>> 移动的坐标系关节如下:\n{coord}")
        self.logger.info(f"已发送移动的关节角度(movj)\n>>>>>> 移动的关节角度如下:\n{pos}")

    def movl(self, robot: int, vel: int, coord: int, pos: List[float]):
        """
        机器人直线运动MOVL

        Args:
            robot(int): 机器人号码
            vel(int): 速度,单位mm/s,1以上的整数, 2~1000 整数
            coord(int): 坐标模式
                - 0:  关节坐标(Joint)
                - 1:  直角坐标(Cart)
                - 2:  工具坐标(Tool)
                - 3:  用户坐标(User)
            pos(List[float]):[1.1,2.2,3.3,4.4,5.5,6.6,7.7]
                - 关节坐标分别代表1-7关节角度值
                - 直角坐标系分别x,y,z,a,b,c。工具、用户同直角。直角坐标系下第七位参数默认为0即可。
        
        Returns:
            None
        """
        cmd_data = {"robot":robot, "vel":vel, "coord":coord, "pos":pos}
        self._send_command(self.CONSTANTS['ROBOT_MOVEMENT']['LINEAR'], cmd_data)
        self.logger.info(f"已发送移动的关节角度(movl)\n>>>>>> 移动的坐标系关节如下:\n{coord}")
        self.logger.info(f"已发送移动的关节角度(movl)\n>>>>>> 移动的关节角度如下:\n{pos}")
    
    def movc(self, robot: int, vel: int, coord: int, isFull:str, posOne: List[float], posTwo: List[float], posThree: List[float]):
        """
        机器人圆弧运动MOVC
        
        Args:
            robot(int): 机器人号码
            vel(int): 速度,单位 mm/s,1以上的整数, 2~1000 整数
            coord(int): 坐标模式
                - 0:  关节坐标(Joint)
                - 1:  直角坐标(Cart)
                - 2:  工具坐标(Tool)
                - 3:  用户坐标(User)
            isFull(str): 
                - "false": MOVC
                - "true": MOVCA(整圆)
            posOne(List[float]): 圆弧起始点 
            posTwo(List[float]):圆弧经过的中间点
            posThree(List[float]): 圆弧的目标点
        
        Returns:
            None
        """
        cmd_data = {"robot":robot, "vel":vel, "coord":coord, "isFull": isFull, "posOne":posOne, "posTwo":posTwo, "posThree":posThree}
        self._send_command(self.CONSTANTS['ROBOT_MOVEMENT']['CIRULAR'], cmd_data)
        self.logger.info(f"已发送移动的关节角度(movc)\n>>>>>> 移动的坐标系关节如下:\n{coord}")
        self.logger.info(f"已发送移动的关节角度(movc)\n>>>>>> 移动的关节角度如下:\n{posOne}, {posTwo}, {posThree}")

    def movs(self, robot: int, vel: int, coord: int, size: int, pos: List[List[float]]):
        """
        机器人样条曲线运动MOVS

        Args:
            robot(int): 机器人号码
            vel(int): 速度,单位mm/s,1以上的整数, 2~1000 整数
            coord(int): 坐标模式
                - 0:  关节坐标(Joint)
                - 1:  直角坐标(Cart)
                - 2:  工具坐标(Tool)
                - 3:  用户坐标(User)
            size(int): 样条曲线的点数目, 要求至少4个点位
            pos(List[float]): 样条曲线的轨迹点位
                            [[1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7],  
                            [1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7], 
                            [1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7], 
                            [1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7]]
                - 关节坐标分别代表1-7关节角度值
                - 直角坐标系分别x,y,z,a,b,c。工具、用户同直角。直角坐标系下第七位参数默认为0即可。
        
        Returns:
            None
        """
        cmd_data = {"robot":robot, "vel":vel, "coord":coord, "size":size, "pos":pos}
        self._send_command(self.CONSTANTS['ROBOT_MOVEMENT']['SPLINE'], cmd_data)
        self.logger.info(f"已发送多点移动指令(movs)\n>>>>>> 移动的关节角度如下:\n{pos}")

    def jog_operation_move(self, axis: int, direction: int):
        """
        执行点动操作
        (注意，点动操作没有返回的命令字数据)
        
        Args:
            axis(int): 代表所要操作的轴, 如 1 代表轴1, 外部轴从8开始
            direction(int): 
                - 1: 正向
                - -1: 反向
        
        Returns:
            None
        """
        cmd_data = {"axis":axis, "direction":direction}
        self._send_command(self.CONSTANTS['JOG_OPERATION']['MOVE'], cmd_data)
        # self.logger.info(f"开始点动操作")

    def jog_operation_stop(self, axis: int):
        """
        停止执行点动操作

        Args:
            axis(int): 代表所要操作的轴, 如 1 代表轴1, 外部轴从8开始
        
        Returns:
            None
        """
        cmd_data = {"axis":axis}
        self._send_command(self.CONSTANTS['JOG_OPERATION']['STOP'], cmd_data)
        self.logger.info(f"停止点动操作")
    
    def jog_jointparameter_set(self, AxisNum: int, MaxSpeed: int, MaxAcc: int):
        """
        设置关节轴点动速度

        Args:
            AxisNum(int): 表示设置的关节轴
            MaxSpeed(int): 关节轴最大点动速度 单位：度°/s
            MaxAcc(int): 关节轴点动加速度 单位：度°/s2

        Returns:
            None
        """
        cmd_data = {"AxisNum":AxisNum,"MaxSpeed":MaxSpeed,"MaxAcc":MaxAcc}
        self._send_command(self.CONSTANTS['JOG_JOINTPARAMETER']['SET'], cmd_data)
        time.sleep(0.5)
        self.logger.info(f"设置关节轴 {AxisNum} 的最大点动速度为{MaxSpeed}, 点动加速度{MaxAcc}")
    
    def jog_jointparameter_inquire(self, AxisNum: int):
        """
        查询关节轴点动速度

        Args:
            AxisNum(int): 表示需要查询的关节轴
        
        Returns:
            AxisNum(int): 表示查询的关节轴
            MaxSpeed(int): 关节轴最大点动速度 单位：度°/s
            MaxAcc(int): 关节轴点动加速度 单位：度°/s2
        """
        cmd_data = {"AxisNum":AxisNum}
        self._send_command(self.CONSTANTS['JOG_JOINTPARAMETER']['INQUIRE'], cmd_data)
        time.sleep(0.1)
        AxisNum = self._return_get('2606AxisNum')
        self.logger.info(f"关节轴编号为：{AxisNum}")
        MaxSpeed = self._return_get('2606MaxSpeed')
        self.logger.info(f"关节轴最大点动速度为：{MaxSpeed}")
        MaxAcc = self._return_get('2606MaxAcc')
        self.logger.info(f"关节轴点动加速度为：{MaxAcc}")
        return AxisNum, MaxSpeed, MaxAcc

    def jog_sensitivity_set(self, Sensitivity: float):
        """
        设置点动灵敏度(默认 0.001)

        Args:
            Sensitivity(float): 点动灵敏度, 单位 度, 范围 0.001 - 1
        
        Returns:
            None
        """
        cmd_data = {"Sensitivity":Sensitivity}
        self._send_command(self.CONSTANTS['JOG_SENSITIVITY']['SET'], cmd_data)
        time.sleep(0.5)
        self.logger.info(f"设置点动灵敏度为: {Sensitivity}")
    
    def jog_sensitivity_inquire(self):
        """
        查询点动灵敏度

        Args:
            None
        
        Returns:
            float: 当前点动灵敏度，单位 度, 范围 0.001 - 1
        """
        self._send_command(self.CONSTANTS['JOG_SENSITIVITY']['INQUIRE'])
        time.sleep(0.1)
        Sensitivity = self.return_status.get('260CSensitivity', 0)
        self.logger.info(f"点动灵敏度为: {Sensitivity} 度")
        return Sensitivity

    def multi_point_move(self, target_vecs: List[List[float]]):
        """
        7000端口运动控制, 命令字  0x9521
        (这个控制端口一定要记得在控制器上建立作业文件, 通过6001端口运行该文件之后才可使用)

        Args:
            target_vecs(List[List[float]]): 
                - 轨迹运动: [[1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7],
                            [1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7],
                            ......]
                - 关节运动: [[1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7]]
        
        Returns:
            None
        """
        cmd_data = { 
            "robot": 1, 
            "clearBuffer": 1, 
            "targetMode": 0, 
            "cfg": { 
                "coord": "ACS", 
                "extMove": 0, 
                "sync": 0, 
                "speed": 100, 
                "acc": 100, 
                "pl": 5, 
                "moveMode": "MOVC" }, 
            "targetVec": [ {"pos": pos} for pos in target_vecs ]  
            }
        
        if self.socket:
            # packed_data = self._crc(FRAME_HEADER, MULTI_POINT, cmd_data.encode("GBK"))
            # self.socket.send(packed_data)
            self._send_command(self.CONSTANTS['MULTI_POINT'], cmd_data)
            self.logger.info(f"已发送多点移动指令(multi_point_move)\n>>>>>> 移动的路径如下:\n{target_vecs}")
        time.sleep(1)
        # return self.return_status.get('9523success', 0)
    
    def set_servo_point_motion_control(self, robot: int, switch: bool):
        """
        开关伺服点位运动控制
        (该功能和"运动控制 0x9521"功能不可同时使用)

        Args:
            robot(int): 机器人号码
            switch(bool): 开关控制命令(注意这个是字符串的true和false)
                - True: 开启伺服点位运动控制
                - False: 关闭伺服点位运动控制
        
        Returns:
            bool: 开关伺服点位运动控制是否成功
                - True: 开启成功
                - False: 开启失败
        """
        cmd_data = {"robot":robot, "switch":switch}
        self._send_command(self.CONSTANTS['SERVOCONTROL']['OPEN'], cmd_data)
        cause = self._return_get('95A3cause')
        cause_map = {# 接收成功时为空
            "dataErr": "接收到的数据错误", 
            "startupErr": "启动失败",
            "busy": "当前通道被占用"
        }
        switch_map = {
            True: "开启",
            False: "关闭"
        }
        self.logger.info(f"开关伺服点位运动控制状态设置: {cause_map.get(cause, '接收成功')}; 状态: {switch_map.get(switch, '未知')}")
        time.sleep(0.5)
        return cause

    def servo_point_motion_control(self, robot: int, end: int, sum: int, count: int, PosVec: List[List[float]]):
        """
        伺服运动控制

        Args:
            robot(int): 机器人号码
            end(int):
                - 1: 停止之前的持续传输,下面的数据可不发
                - 0: 可不发 end 参数值
            sum(int): 总共要发的帧数
            count(int): 当前为第几帧
            PosVec(List[List[float]]): 机器人的关节角度
                - [[1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7],
                    [1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7],
                    ......]

        Returns:
            tuple[str, bool]:
                - str:  
                    - "": 接收成功
                    - "notStart": 未开启服点位运动控制模式
                    - "dataErr": 数据错误
                    - "termination": 发送端终止了正在传输的数据
                    - "cacheFull": 缓存区已满(最大缓存 6 条轨迹)
                - bool: 判断是否接收成功, 用于后续判断
                    - True: 成功
                    - False: 失败
        """
        cmd_data = {
            "robot":robot, 
            "end":end, 
            "sum":sum, 
            "count":count, 
            "PosVec":PosVec
            }
        
        self._send_command(self.CONSTANTS['SERVOCONTROL']['MOVE'], cmd_data)
        cause = self._return_get('95A6cause')
        if cause == "notStart":
            self.logger.info(f'未开启服点位运动控制模式')
            return cause, False
        elif cause == "dataErr":
            self.logger.info(f'数据错误')
            return cause, False
        elif cause == "termination":
            self.logger.info(f'发送端终止了正在传输的数据')
            return cause, False
        elif cause == "cacheFull":
            self.logger.info(f'缓存区已满(最大缓存 6 条轨迹)')
            return cause, False
        elif cause == None:
            return cause, True