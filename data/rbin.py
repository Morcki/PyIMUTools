import struct


with open("./Data1_PureINS.bin",'rb') as f:
    for i in range(100):
        if i % 10 == 0:
            print('=======')
        data = f.read(8)
        data_float = struct.unpack("d",data)
        print(data_float[0])
        


