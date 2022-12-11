#ifndef SA818_CONTROLLER_H
#define SA818_CONTROLLER_H

class SA818Controller {
    private:
        SA818* sa = 0;
        byte type_ = Model::SA_818;
        byte band_ = Band::VHF;

        byte bw_;
        float tx_f_;
        float rx_f_;
        int tx_sub_;
        byte sq_;
        int rx_sub_;

        bool scanning = false;

        String ctcss(int);
        float loopScan(float, bool, long = millis(), float = 0, long = 60000);

    public:
        SA818Controller(SA818*);
        ~SA818Controller();
        void setModel(Model = Model::SA_818);
        void setBand(Band = Band::VHF);
        String response();
        String result();

        void setBW(byte);
        void setTXF(float);
        void setRXF(float);
        void setTXSub(int);
        void setSQ(byte);
        void setRXSub(int);
        bool update();

        bool connect();
        bool setGroup(byte, float, float, int, byte, int);
        bool scan(float);
        bool setVolume(byte);
        bool setFilter(byte, byte, byte);
        bool openTail();
        bool closeTail();
        String rssi();
        String version();

        float next(float, float = 0, long = 60000);
        float previous(float, float = 0, long = 60000);
        int scanlist(float[], int, float[]);
        void stopScan();
};

#endif


SA818Controller::SA818Controller(SA818* sa818) : sa(sa818) {}

SA818Controller::~SA818Controller() {
    sa = 0;
}

void SA818Controller::setModel(Model type) {
    type_ = type;
}

void SA818Controller::setBand(Band band) {
    band_ = band;
}

String SA818Controller::response() {
    return sa->response()->raw;
}
String SA818Controller::result() {
    return sa->response()->res;
}

void SA818Controller::setBW(byte bw) {
    bw_ = bw;
}

void SA818Controller::setTXF(float tx) {
    tx_f_ = tx;
}

void SA818Controller::setRXF(float rx) {
    rx_f_ = rx;
}

void SA818Controller::setTXSub(int tx) {
    tx_sub_ = tx;
}

void SA818Controller::setSQ(byte sq) {
    sq_ = sq;
}

void SA818Controller::setRXSub(int rx) {
    rx_sub_ = rx;
}

bool SA818Controller::update() {
    return setGroup(bw_, tx_f_, rx_f_, tx_sub_, sq_, rx_sub_);
}

String SA818Controller::ctcss(int sub) {
    String ssub = String(abs(sub));
    if(sub < 0)
        ssub += "N";
    if(sub >= 23)
        ssub += "I";

    String res = ssub;
    for (int i = ssub.length(); i < 4; i++) {
        res = "0" + res;
    }
    return String(res);
}

bool SA818Controller::connect() {
    return sa->send("AT+DMOCONNECT");
}

bool SA818Controller::setGroup(byte bw, float tx_f, float rx_f, int tx_sub, byte sq, int rx_sub) {
    bw_ = bw;

    setBW(bw);
    setTXF(tx_f);
    setRXF(rx_f);
    setTXSub(tx_sub);
    setSQ(sq);
    setRXSub(rx_sub);

    String b = String(bw);
    String tf = String(tx_f, 4);
    String rf = String(rx_f, 4);
    String tsub = ctcss(tx_sub);
    String s = String(sq);
    String rsub = ctcss(rx_sub);
    char* params[] = {
        b.c_str(),
        tf.c_str(),
        rf.c_str(),
        tsub.c_str(),
        s.c_str(),
        rsub.c_str()
    };
    return sa->send("AT+DMOSETGROUP", 6, params);
}

bool SA818Controller::scan(float freq) {
    String cmd = String("S+") + String(freq, 4);
    return sa->send(cmd.c_str());
}

bool SA818Controller::setVolume(byte vol) {
    String v =  String(vol);
    char* params[] = { v.c_str() };
    return sa->send("AT+DMOSETVOLUME", 1, params);
}

bool SA818Controller::setFilter(byte emph, byte high, byte low) {
    String e = String(emph);
    String h = String(high);
    String l = String(low);
    char* params[] = { e.c_str(), h.c_str(), l.c_str() };
    return sa->send("AT+SETFILTER", 3, params);
}

bool SA818Controller::openTail() {
    return sa->send("AT+SETTAIL=1");
}

bool SA818Controller::closeTail() {
    return sa->send("AT+SETTAIL=0");
}

String SA818Controller::rssi() {
    String cmd = "RSSI?";
    if(type_ == Model::SA_868)
        cmd = "AT+RSSI?";

    if(sa->send(cmd.c_str()))
        return sa->response()->res;
    return sa->response()->raw;
}


String SA818Controller::version() {
    if(sa->send("AT+VERSION"))
        return sa->response()->res;
    return sa->response()->raw;
}

float SA818Controller::loopScan(float mhz, bool dir, long timer, float stop, long timeout) {
    // | GHz |     |     | MHz ||     |     | KHz ||     |     | Hz |
    // |     |     |     |     ||     |     |   1 ||   0 |   0 |  0 |
    // |     |     |     |   1 ||   0 |   0 |   0 ||   0 |   0 |  0 |
    // |   1 |   0 |   0 |   0 ||   0 |   0 |   0 ||   0 |   0 |  0 |
    // |     |   1 |   4 |   5 ||   7 |   5 |   0 ||   0 |   0 |  0 |
    // |     |     |     |     ||     |   1 |   2 ||   5 |   0 |  0 |

    if(stop == 0)
        stop = mhz;

    float f = mhz * 1000; // MHz to KHz

    float khz = 12.5; // 12.5K
    if(bw_ == 1) 
        khz = 25; // 25K

    if(dir) f = f + khz;
    else f = f - khz;

    // Out of frequencies bounds
    float top = 174000; // VHF KHz top limit
    float bottom = 134000;
    if(band_ == Band::UHF) {
        top = 480000; // UHF
        bottom = 400000;
    }

    if (dir) {
        f = f + khz;

        if (f > top) f = bottom; // out of top bound
    }
    else {
        f = f - khz;

        if (f < bottom) f = top; // out of bottom bound
    }
    f = f / 1000;

    if(scan(f)) return f;
    if(f == stop || millis() - timer >= timeout) return mhz;
    else return loopScan(f, dir, timer, stop, timeout);
}

float SA818Controller::next(float mhz, float stop, long timeout) {
    if(stop == 0)
        stop = mhz;
    return loopScan(mhz, true, millis(), stop, timeout);
}

float SA818Controller::previous(float mhz, float stop, long timeout) {
    if(stop == 0)
        stop = mhz;
    return loopScan(mhz, false, millis(), stop, timeout);
}

int SA818Controller::scanlist(float list[], int size, float ret[]) {
    int r = 0;
    for(int i = 0; i < size; i++) {
       if(scan(list[i])) 
            ret[r++] = list[i];
    }

    return r;
}
