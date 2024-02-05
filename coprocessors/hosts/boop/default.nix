{ ... }:

{
  config = {
    networking = {
      hostName = "boop";

      robotNetwork = {
        interface = "end1";
        address = "10.36.36.11";
      };
    };

    system.stateVersion = "23.05";
  };
}
