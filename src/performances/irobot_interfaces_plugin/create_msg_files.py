from typing import List
import os
import shutil

def get_msg_name(size: int, unit: str) -> str:
    return f"Stamped{size}{unit}.msg"

def get_msg_content(byte_size: int) -> List[str]:
    content = ["", ""]
    content[0] = "performance_test_msgs/PerformanceHeader header\n"
    content[1] = "byte[" + str(byte_size) + "] data"
    return content

def convert_mb_to_byte(mb: int) -> int:
    return 1024 * 1024 * mb
    
def convert_kb_to_byte(kb: int) -> int:
    return 1024 * kb

def convert_size_to_byte(size: int, unit: str):
    if unit.upper() == 'MB':
        size = convert_kb_to_byte(size)
    if unit.upper() == 'KB':
        size = convert_kb_to_byte(size)
    return size

if __name__ == '__main__':
    ################################################
    # fill in your message sizes here 
    #############################################
    msg_sizes = [
        (32, 'b'),
        (256, 'b'),
        (512, 'b'),
        (1024, 'b'),
        (4096, 'b'),
        (16384, 'b'),
        (63000, 'b')
    ]

    ############################################
    ######################################
    for msg_size in msg_sizes:
        size = msg_size[0]
        unit = msg_size[1]

        msg_name = get_msg_name(size, unit)
        msg_content = get_msg_content(convert_size_to_byte(size, unit))

        with open(os.path.join('msg', msg_name), 'w') as f:
            f.writelines(msg_content)

        with open('add_cmake.txt', 'a') as f:
            f.writelines('msg/' + msg_name + '\n')

            
