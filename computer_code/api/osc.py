
from pythonosc import udp_client


class OSC:
    def __init__(self,ip="127.0.0.1",port=5005):
        self.client = udp_client.SimpleUDPClient(ip, port)
        
    def sendOSC_ObjectPoints(self, object_points):
        
        for i, object_point in enumerate(object_points):
            #print([i,float(object_point[0]),float(object_point[1]),float(object_point[2])])
            self.client.send_message("/objectPoint", [i,float(object_point[0]),float(object_point[1]),float(object_point[2])])