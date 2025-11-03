// @ts-nocheck
import { useState } from "react";
import { droneData } from "../../data/droneData";
import { DroneCard } from "./DroneCard";
import { Sheet, SheetContent, SheetTrigger } from "../../components/ui/sheet";
import { ScrollArea } from "../../components/ui/scroll-area";
import { useWebSocket } from "../../hooks/useWebSocket";
import { PanelsRightBottom } from "lucide-react";
import { Button } from "../ui/button";
import { useFetch } from "../../api/useFetch";
import { DroneInfo } from "../../types/drone";
import { WebSocketDroneData,DroneData } from "./types";
import { Card, CardContent } from "../../components/ui/card";


const DroneCardsSidebar = () => {

  const { droneData } = useWebSocket();
//  console.log(droneData)
  const formatDroneData = (name: string, data: WebSocketDroneData): DroneData => ({
    id: name,  
    name: name.charAt(0).toUpperCase() + name.slice(1),  
    image: "https://picsum.photos/400/300", 
    status: {
      connection: {
        color: data.conn ? "bg-green-500" : "bg-red-500",
        text: data.conn ? "Connected" : "Disconnected"
      },
      armed: {
        color: data.armed ? "bg-green-500" : "bg-red-500",
        text: data.armed ? "Armed" : "Disarmed"
      },
      flight: {
        color: data.in_air ? "bg-blue-500" : "bg-yellow-500",
        text: data.in_air ? "In Air" : "On Ground"
      },
      time: "Offboard", 
      speed: "25km/h",  
      Bettry:"50 Mins"
    }
  });

  const NoDronesCard = () => (
    <Card className="bg-[#3535354c] rounded-[10px] border border-solid border-[#ffffff1a] backdrop-blur-[7px]">
      <div className="p-[18px] pb-0 flex justify-between items-center">
        <span className="font-normal text-white text-xs">Status</span>
      </div>

      <CardContent
        className="p-5 bg-[#1a1a1a] rounded-[20px] mt-2 h-[196px] relative flex items-center justify-center"
      >
        <div className="text-center">
          <div className="text-white/70 text-sm mb-2">No Drones Found</div>
          <div className="text-white/50 text-xs">Waiting for drone connections...</div>
        </div>
      </CardContent>
    </Card>
  );



  return (
    <>
      {/* Visible on larger screens, hidden on mobile */}
      <div className="hidden md:block">
        <ScrollArea className="md:h-[calc(100vh-110px)]">
          <div className="space-y-4 pr-4">
          {(!droneData?.drones || Object.keys(droneData.drones).length === 0) ? (
              <NoDronesCard />
            ) : (
              Object.entries(droneData.drones).map(([droneName, droneInfo]) => (
                <DroneCard 
                  key={droneName} 
                  drone={formatDroneData(droneName, droneInfo)} 
                />
              ))
            )}

          </div>
        </ScrollArea>
      </div>

      {/* Sheet for mobile devices */}
      {/* <div className="md:hidden">
        <Sheet open={open} onOpenChange={setOpen}>
          <SheetTrigger asChild>
            <Button
              variant="outline"
              size="icon"
              className=" bg-[#1a1a1a80] backdrop-blur-md hover:bg-[#ffffff20] border-[#ffffff20]"
            >
              <PanelsRightBottom className="h-5 w-5 text-white" />
            </Button>
          </SheetTrigger>
          <SheetContent
            side="right"
            className="w-[85vw] sm:w-[350px] bg-[#353535] border-none"
          >
            <div className="py-6">
              <h2 className="text-lg font-semibold text-white mb-4">Drones</h2>
              <ScrollArea className="h-[calc(100vh-150px)]">
                {data && !isLoading ? (
                  <div className="space-y-4 pr-4">
                    {data?.map((drone) => (
                      <DroneCard
                        key={drone.id}
                        drone={{
                          id: drone.id,
                          name: drone.name,
                          image: "https://picsum.photos/400/300",
                          status: {
                            connection: {
                              color: drone.status.connected
                                ? "bg-green-500"
                                : "bg-red-500",
                              text: drone.status.connected
                                ? "Connected"
                                : "Disconnected",
                            },
                            armed: {
                              color: drone.status.armed
                                ? "bg-green-500"
                                : "bg-red-500",
                              text: drone.status.armed ? "Armed" : "Disarmed",
                            },
                            flight: {
                              color:
                                drone.status.flight_mode === "AUTO"
                                  ? "bg-blue-500"
                                  : "bg-yellow-500",
                              text:
                                drone.status.flight_mode === "AUTO"
                                  ? "Auto"
                                  : "Manual",
                            },
                            time: drone.battery + "mins",
                            speed: drone.velocity + "km/h",
                          },
                        }}
                      />
                    ))}
                  </div>
                ) : (
                  <div className="space-y-4 pr-4">
                    {droneData.map((drone) => (
                      <DroneCard key={drone.id} drone={drone} />
                    ))}
                  </div>
                )}
              </ScrollArea>
            </div>
          </SheetContent>
        </Sheet>
      </div> */}
    </>
  );
};

export default DroneCardsSidebar;
