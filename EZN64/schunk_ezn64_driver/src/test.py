import socket
host='192.168.5.243' 
buf=1024
port=1025
addr=(host,port)
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.bind(('', port))

#   #aaDnd

s.sendto('#010\r', (host,port))
indata, inaddr = s.recvfrom(buf)
print '10: ' + indata
s.sendto('#011\r', (host,port))
indata, inaddr = s.recvfrom(buf)
print '11: ' + indata
s.sendto('#012\r', (host,port))
indata, inaddr = s.recvfrom(buf)
print '12: ' + indata
s.sendto('#013\r', (host,port))
indata, inaddr = s.recvfrom(buf)
print '13: ' + indata
s.sendto('#014\r', (host,port))
indata, inaddr = s.recvfrom(buf)
print '14: ' + indata
s.sendto('#015\r', (host,port))
indata, inaddr = s.recvfrom(buf)
print '15: ' + indata

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
