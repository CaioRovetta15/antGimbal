import socket
import os
import threading as th
import time

in1 = 0
in2 = 0

def send_joint_angles(angle1, angle2):
    global in1, in2
    in1 = angle1
    in2 = angle2
    return

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

def __sendCommand(conn, angle1, angle2, lastMsg, debug=False) :
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
        msg = __formatMessage(angle1, angle2)
    except ValueError as e:
        print(e)
        msg = lastMsg
    if debug : print( "Command sent: " + msg, end="" )
    # Send message
    data = msg.encode('utf-8')
    conn.sendall(data)
    lastMsg = str(msg)
    return True, lastMsg

def __extract_ip():
    st = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:       
        st.connect(('10.255.255.255', 1))
        IP = st.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        st.close()
    return IP

def connectESP32(port, debug=False) :
    """Connect to esp32 via socket."""
    # Get IP address dynamically
    host = __extract_ip()
    lastMsg = "090090\n" # Default value: 90, 90
    # Connection 
    print("--- Antenna Controller Server ---")
    print(f"Started Server at {host} port {port}")

    # Create socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        conn, addr = s.accept()
        with conn:
            # Connected successfully
            print(f"Connected by {addr}")
            looping = True
            while( looping ) :
                looping, lastMsg = __sendCommand(conn, in1, in2, lastMsg, debug=True)
            conn.close()
        s.close()

# makes thread safe socket connection to send commands to esp32

if __name__ == "__main__" :
    thread = th.Thread(target=connectESP32, args=(8890,))
    thread.start()