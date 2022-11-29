import socket
import os

def __formatMessage(angle1, angle2) :
    # Numeric check
    if not angle1.isnumeric() or not angle2.isnumeric() :
        raise ValueError( f"Not a number! angle1:{angle1} angle2:{angle2}")
        return
    # Range check
    if (int(angle1) < 0 or int(angle1) > 180) or (int(angle2) < 0 or int(angle2) > 180) :
        raise ValueError( f"Not in range! angle1:{angle1} angle2:{angle2}")
        return
    # Format number
    angles = [angle1, angle2]
    for i in range( 2 ) :
        if int(angles[i])  <=  9 :
            angles[i] = "00" + angles[i]
        elif int(angles[i]) <= 99 :
            angles[i] = "0" + angles[i]
    return (angles[0]+angles[1]+"\n")  

def sendCommand(conn, angle1, angle2, lastMsg, debug=False) :
    """Send commands to servo motor via socket.
    conn: connection session with esp32
    angle1, angle2: servo's target angles
    lastMsg: last command sent to esp32
    """
    angle1 = str(angle1).lower()
    angle2 = str(angle2).lower()
    # Check exit command
    if angle1 == "exit" or angle2 == "exit":
        conn.sendall("exit\n".encode('utf-8'))
        return False, lastMsg
    # Check input and format it
    try :
        msg = __formatMessage(str1, str2)
    except ValueError as e:
        print(e)
        msg = lastMsg
    if debug : print( "Command sent: " + msg, end="" )
    # Send message
    data = msg.encode('utf-8')
    conn.sendall(data)
    lastMsg = str(msg)
    return True, lastMsg

if __name__ == "__main__" :
    HOST = "192.168.0.115" 
    PORT = 8890
    lastMsg = "090090\n" # Default value: 90, 90
    # Connection 
    print("--- Antenna Controller Server ---")
    print(f"Started Server at {HOST} port {PORT}")
    # Socket session
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        with conn:
            # Connected successfully
            print(f"Connected by {addr}")
            looping = True
            while( looping ) :
                # Write angle from terminal <- MUDAR ISSO
                str1 = input()
                str2 = input()
                looping, lastMsg = sendCommand(conn, str1, str2, lastMsg, debug=True)
            conn.close()
        s.close()