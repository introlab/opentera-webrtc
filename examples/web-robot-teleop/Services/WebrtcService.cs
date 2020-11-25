using Microsoft.JSInterop;
using System.Collections.Generic;

/**

const SignallingServerConfiguration = {
      url: 'http://localhost:8080',
      name: nameInput.value,
      data: {}, // Client custom data
      room: 'chat',
      password: passwordInput.value
    };

    */

namespace web_robot_teleop.Services
{
    public class SignallingServerConfiguration
    {
        public string url = "http://locahost:8080";
        public string name = "operator";
        Dictionary<string, object> data = new Dictionary<string, object>();
        public string room = "chat";
        public string password = "";
    }

    public class WebrtcService
    {
        private readonly IJSRuntime js;

        public SignallingServerConfiguration SignalingConfig = new SignallingServerConfiguration();
        private Dictionary<string, object> streamConfiguration = new Dictionary<string, object>() {{"isSendOnly", false}};
        private Dictionary<string, object> dataChannelConfiguration = new Dictionary<string, object>();
        private Dictionary<string, object> iceServers = new Dictionary<string, object>();

        public WebrtcService(IJSRuntime jsRuntime)
        {
            js = jsRuntime;
        }

        public async void Connect()
        {
            var client = js.InvokeAsync<
        }

        public void Disconnect()
        {

        }
    }
}
