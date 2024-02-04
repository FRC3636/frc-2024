{ config, lib, ... }:

let
  robotNetworkConfig = config.networking.robotNetwork;
in
{
  options = {
    networking.robotNetwork = {
      interface = lib.mkOption {
        type = lib.types.str;
        description = "The network interface to use for the robot network";
      };

      address = lib.mkOption {
        type = lib.types.str;
        description = "The address to use for the robot network.";
      };
    };
  };

  config = {
    networking = {
      nameservers = [ "1.1.1.1" "1.0.0.1" ];

      interfaces."${robotNetworkConfig.interface}" = {
        ipv4 = {
          addresses = [{ address = robotNetworkConfig.address; prefixLength = 24; }];
          routes = [{ address = "0.0.0.0"; prefixLength = 0; via = "10.36.36.1"; }];
        };
      };
    };
  };
}