import zmq
import pickle

# Create a ZeroMQ PULL socket
context = zmq.Context()
socket = context.socket(zmq.PULL)
socket.connect("tcp://127.0.0.1:5555")  # Connect to the sender

while True:
    # Receive the pickled data
    message = socket.recv()
    data = pickle.loads(message)  # Deserialize with pickle


    print(data)