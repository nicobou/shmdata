#include <iostream>
#include "vrpn_Analog.h"
#include "vrpn_Button.h"
using namespace std;

void VRPN_CALLBACK handle_analog(void* userData, const vrpn_ANALOGCB a) {
  cout << "Analog : ";

  for (int i = 0; i < a.num_channel; i++) {
    cout << a.channel[i] << " ";
  }

  cout << endl;
}

void VRPN_CALLBACK handle_button(void* userData, const vrpn_BUTTONCB b) {
  static int count = 0;
  static int buttonstate = 1;

  if (b.state != buttonstate) {
    std::cout << "button " << b.button << " is in state " << b.state << std::endl;
    buttonstate = b.state;
    count++;
  }
}

int main(int argc, char* argv[]) {
  vrpn_Connection* c = vrpn_get_connection_by_name("localhost:3884");
  if (c->doing_okay()) {
    std::cout << "ok" << std::endl;
  } else {
    std::cout << "not ok" << std::endl;
  }
  if (c->connected()) {
    std::cout << "connected" << std::endl;
  } else {
    std::cout << "not connected" << std::endl;
  }
  vrpn_Analog_Remote* vrpnAnalog = new vrpn_Analog_Remote("joy@localhost:3884", c);
  vrpn_Button_Remote* vrpnButton = new vrpn_Button_Remote("joy@localhost:3884", c);

  vrpnAnalog->register_change_handler(0, handle_analog);
  vrpnButton->register_change_handler(0, handle_button);

  while (1) {
    c->mainloop();
    vrpnAnalog->mainloop();
    vrpnButton->mainloop();
  }

  return 0;
}
