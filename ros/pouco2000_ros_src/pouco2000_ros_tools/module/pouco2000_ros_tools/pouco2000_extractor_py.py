from abc import abstractmethod
from pouco2000_ros_msgs.msg import Controller 

class Extractor(object):
    """!
    Extract data from msg controller 
    """

    def __init__(self,index):
        self.__index = index
        self.__data_previous = None 
        self._data = None 
        
        
    @abstractmethod
    def __extract_field(self,msg):
        """!
        extract from controller msg the field, method need by override by child class  
        """
        raise NotImplementedError

    def get_data(self):
        """!
        get the current data extract from controller msg  
        """
        return self._data

    def extract(self,msg):
        """!
        extract the data from controller msg 
        """
        data_field = self._extract_field(msg)
        if(len(data_field.data) > self.__index):
            self._data = data_field.data[self.__index]
            return True
        return False 

    def extract_only_change(self,msg):
        """!
        extract the data only if the data is different of the previous msg 
        """
        if(self.extract(msg)):
            if(self._data != self.__data_previous):
                self.__data_previous = self._data
                return True
        return False  
    

class ExtractorButton(Extractor):
    """!
    Extractor for buttons field
    """

    def __init__(self,index):
        super().__init__(index)

    def _extract_field(self,msg):
        return msg.buttons

    def is_push(self,msg):
        """!
        inform if the button is push or not 
        """
        return self.extract_only_change(msg) and self._data
    
class ExtractorSwitchOnOff(Extractor):
    """!
    Extractor for switchs_on_off field
    """

    def __init__(self,index):
        super().__init__(index)

    def _extract_field(self,msg):
        return msg.switchs_on_off

    def is_on(self,msg):
        """!
        inform if the switch is on 
        """
        return self.extract_only_change(msg) and self._data

class ExtractorSwitchMode(Extractor):
    """!
    Extractor for switchs_mode field
    """

    def __init__(self,index):
        super().__init__(index)

    def _extract_field(self,msg):
        return msg.switchs_mode

    def is_mode(self,msg,mode):
        """!
        inform is the switch is on mode "mode"
        """
        return self.extract_only_change(msg) and self._data == mode

class ExtractorPotentiometerCircle(Extractor):
    """!
    Extractor for potentiometers_circle field
    """

    def __init__(self,index):
        super().__init__(index)

    def _extract_field(self,msg):
        return msg.potentiometers_circle

class ExtractorPotentiometerSlider(Extractor):
    """!
    Extractor for potentiometers_slider field
    """

    def __init__(self,index):
        super().__init__(index)

    def _extract_field(self,msg):
        return msg.potentiometers_slider


class HandleExtractors: 
    """!
    Handle all extractors avoiding to init many Extractor objects 
    """

    def __init__(self):
        self.map_extractors_buttons = {}
        self.map_extractors_switchs_onoff = {}
        self.map_extractors_switchs_modes = {}
        self.map_extractors_potentiometers_circle = {} 
        self.map_extractors_potentiometers_slider = {}

    def __init_element(self,index, map_extractors,class_extractor):
        if(not(index in map_extractors)):
            map_extractors[index] = class_extractor(index)
        
    def get_button(self,index):
        self.__init_element(index,self.map_extractors_buttons,ExtractorButton)
        return self.map_extractors_buttons[index]
    
    def get_switch_onoff(self,index):
        self.__init_element(index,self.map_extractors_switchs_onoff,ExtractorSwitchOnOff)
        return self.map_extractors_switchs_onoff[index]

    def get_switch_modes(self,index):
        self.__init_element(index,self.map_extractors_switchs_modes,ExtractorSwitchMode)
        return self.map_extractors_switchs_modes[index]

    def get_potentiometer_circle(self,index):
        self.__init_element(index,self.map_extractors_potentiometers_circle,ExtractorPotentiometerCircle)
        return self.map_extractors_potentiometers_circle[index]

    def get_potentiometer_slider(self,index):
        self.__init_element(index,self.map_extractors_potentiometers_slider,ExtractorPotentiometerSlider)
        return self.map_extractors_potentiometers_slider[index]


