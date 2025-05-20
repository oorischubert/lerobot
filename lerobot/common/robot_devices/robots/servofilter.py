class ServoFilter:
    def __init__(self, configs):
        """
        Initialize the ServoFilter with a config dictionary.
        The config should be a dict mapping motor IDs (as strings) to:
        {
            "safePoint": int,
            "borderMin": int,
            "borderMax": int
        }
        """
        self.configs = configs
    
    def borderDecellerator(self,value,border,speed):
        """When value nears border, decellerates at speed."""
        return value # placeholder

    def __call__(self, values):
        """
        Filter the given list of values to ensure they stay within configured borders.
        Each value in the list is clamped to [borderMin, borderMax] for the corresponding motor ID.
        Assumes values[i] corresponds to motor with ID i (as string).
        """
        filtered = []
        
        for i, value in enumerate(values):
            
            str_id = str(i)
            if str_id in self.configs:
                conf = self.configs[str_id]
                min_val = conf["borderMin"]
                max_val = conf["borderMax"]
                safe = conf["safePoint"]
                
                if min_val == max_val:
                    # Single-border case: decide if it’s an upper or lower bound
                    if safe > min_val:
                        # Border is lower limit
                        filtered_value = max(min_val, value)
                    else:
                        # Border is upper limit
                        filtered_value = min(max_val, value)
                else:
                    # Two borders — clamp normally
                    filtered_value = max(min_val, min(max_val, value))

                filtered.append(filtered_value)
            else:
                filtered.append(value)
                
        return filtered
