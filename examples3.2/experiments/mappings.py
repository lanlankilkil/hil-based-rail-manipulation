# from experiments.ram_insertion.config import TrainConfig as RAMInsertionTrainConfig
# from experiments.usb_pickup_insertion.config import TrainConfig as USBPickupInsertionTrainConfig
from experiments.object_handover.config import TrainConfig as ObjectHandoverTrainConfig
from experiments.pick_ironroad.config import TrainConfig as PickIronRoadTrainConfig
from experiments.egg_flip.config import TrainConfig as EggFlipTrainConfig

CONFIG_MAPPING = {
                # "ram_insertion": RAMInsertionTrainConfig,
                # "usb_pickup_insertion": USBPickupInsertionTrainConfig,
                "object_handover": ObjectHandoverTrainConfig,
                "pick_ironroad": PickIronRoadTrainConfig,
                # "egg_flip": EggFlipTrainConfig,
               }