import socket
host='10.0.0.1' 
buf=1024
port=1025
addr=(host,port)
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.bind(('', port))

#   #aaDnd

s.sendto('#016\r', (host,port))
indata, inaddr = s.recvfrom(buf)
print indata
s.sendto('#017\r', (host,port))
indata, inaddr = s.recvfrom(buf)
print indata

#s.sendto('#01D01\r', (host,port))
#indata, inaddr = s.recvfrom(buf)
#print indata
##raw_input()
#s.sendto('#01D11\r', (host,port))
#indata, inaddr = s.recvfrom(buf)
#print indata

#s.sendto('#016\r', (host,port))
#indata, inaddr = s.recvfrom(buf)
#print indata
##s.sendto('#017\r', (host,port))
##indata, inaddr = s.recvfrom(buf)
##print indata


#s.sendto('#01D00\r', (host,port))
#indata, inaddr = s.recvfrom(buf)
#print indata
##s.sendto('#01D10\r', (host,port))
##indata, inaddr = s.recvfrom(buf)
##print indata

#s.sendto('#016\r', (host,port))
#indata, inaddr = s.recvfrom(buf)
#print indata
#s.sendto('#017\r', (host,port))
#indata, inaddr = s.recvfrom(buf)
#print indata
