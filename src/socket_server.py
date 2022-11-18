import socket
import os

def formatMessage(alpha, beta) :
    if not alpha.isnumeric() or not beta.isnumeric() :
        raise ValueError( f"Not a number! Alpha:{alpha} Beta:{beta}")
        return
    if (int(alpha) < 0 or int(alpha) > 180) or (int(beta) < 0 or int(beta) > 180) :
        raise ValueError( f"Not in range! Alpha:{alpha} Beta:{beta}")
        return

    angles = [alpha, beta]

    for i in range( 2 ) :
        if int(angles[i])  <=  9 :
            angles[i] = "00" + angles[i]
        elif int(angles[i]) <= 99 :
            angles[i] = "0" + angles[i]

    return (angles[0]+angles[1]+"\n")    

HOST = "192.168.0.115"  # Standard loopback interface address (localhost)
PORT = 8890  # Port to listen on (non-privileged ports are > 1023)
DEFAULT_POSITION = "090090\n"
lastMsg = DEFAULT_POSITION

print("--- Antenna Controller Server ---")
print(f"Started Server at {HOST} port {PORT}")

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        while( True ) :
            str1 = input()
            if str1.lower() == "exit" :
                conn.sendall("exit\n".encode('utf-8'))
                exit(1)
            str2 = input()
            try :
                msg = formatMessage(str(str1), str(str2))
            except ValueError as e:
                print(e)
                msg = lastMsg
            print( "Command sent: " + msg, end="" )
            data = msg.encode('utf-8')
            conn.sendall(data)
            lastMsg = str(msg)

        conn.close()

    s.close()

