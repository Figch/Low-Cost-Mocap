
from pythonosc import udp_client


class OSC:
    def __init__(self,ip="127.0.0.1",port=5005):
        self.client = udp_client.SimpleUDPClient(ip, port)
        
    def sendOSC_ObjectPoints(self, object_points):
        
        for i, object_point in enumerate(object_points):
            #print([i,float(object_point[0]),float(object_point[1]),float(object_point[2])])
            self.client.send_message("/marker", [i,float(object_point[0]),float(object_point[1]),float(object_point[2])])

    def sendOSC_filteredObjects(self, filtered_points):
        for i, object_point in enumerate(filtered_points):
            pos=object_point.pos
            vel=object_point.vel
            self.client.send_message("/object", [object_point.index,float(pos[0]),float(pos[1]),float(pos[2]),float(vel[0]),float(vel[1]),float(vel[2]),float(object_point.heading)])
            