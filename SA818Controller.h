#ifndef SA818_CONTROLLER_H
#define SA818_CONTROLLER_H

class SA818Controller {
    private:
        SA818* sa = 0;

        byte bw_;
        float tx_f_;
        float rx_f_;
        int tx_sub_;
        byte sq_;
        int rx_sub_;

        bool success(Response*);
        String ctcss(int);
        float loopScan(float, bool, float = 0);

    public:
        SA818Controller(SA818*);
        ~SA818Controller();

        bool connect();
        bool setGroup(byte, float, float, int, byte, int);
        bool updateGroup();

        void setBW(byte);
        void setTXF(float);
        void setRXF(float);
        void setTXSub(int);
        void setSQ(byte);
        void setRXSub(int);

        bool connect();
        bool setGroup(byte, float, float, int, byte, int);
        bool scan(float);
        bool setVolume(int);
        bool setFilter(int, int, int);
        bool openTail();
        bool closeTail();
        String rssi();
        String version();

        float next(float, float = 0);
        float previous(float, float = 0);
};

#endif


SA818Controller::SA818Controller(SA818* sa818) : sa(sa818) {}

SA818Controller::~SA818Controller() {
    sa = 0;
}

bool SA818Controller::connect() {
    return sa->connect();
}

bool SA818Controller::setGroup(byte bw, float tx_f, float rx_f, int tx_sub, byte sq, int rx_sub) {
    setBW(bw);
    setTXF(tx_f);
    setRXF(rx_f);
    setTXSub(tx_sub);
    setSQ(sq);
    setRXSub(rx_sub);
    return sa->setGroup(bw_, tx_f_, rx_f_, tx_sub_, sq_, rx_sub_);
}

bool SA818Controller::updateGroup() {
    return sa->setGroup(bw_, tx_f_, rx_f_, tx_sub_, sq_, rx_sub_);
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

bool SA818Controller::success(Response* resp) {
    if(resp->complete) {
        lastResponse_ = *resp;

        if(resp->res != "") {
            if(resp->res.equals("1")) return false;
            return true;
        }

        if(resp->raw.equals("+DMOERROR")) return false;
    }

    return false;
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
    return send("AT+DMOCONNECT");
}

bool SA818Controller::setGroup(byte bw, float tx_f, float rx_f, int tx_sub, byte sq, int rx_sub) {
    bw_ = bw;

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
    return send("AT+DMOSETGROUP", 6, params);
}

bool SA818Controller::scan(float freq) {
    String cmd = String("S+") + String(freq, 4);
    return send(cmd.c_str());
}

bool SA818Controller::setVolume(int vol) {
    String v =  String(vol);
    char* params[] = { v.c_str() };
    return send("AT+DMOSETVOLUME", 1, params);
}

bool SA818Controller::setFilter(int emph, int high, int low) {
    String e = String(emph);
    String h = String(high);
    String l = String(low);
    char* params[] = { e.c_str(), h.c_str(), l.c_str() };
    return send("AT+SETFILTER", 3, params);
}

bool SA818Controller::openTail() {
    return send("AT+SETTAIL=1");
}

bool SA818Controller::closeTail() {
    return send("AT+SETTAIL=0");
}

String SA818Controller::rssi() {
    String cmd = "RSSI?";
    if(type_ == SA_868)
        cmd = "AT+RSSI?";

    if(send(cmd.c_str()))
        return lastResponse_.res;
    return response();
}


String SA818Controller::version() {
    if(send("AT+VERSION"))
        return lastResponse_.res;
    return response();
}

float SA818Controller::loopScan(float mhz, bool dir, float khz) {
    // | GHz |     |     | MHz ||     |     | KHz ||     |     | Hz |
    // |     |     |     |     ||     |     |   1 ||   0 |   0 |  0 |
    // |     |     |     |   1 ||   0 |   0 |   0 ||   0 |   0 |  0 |
    // |   1 |   0 |   0 |   0 ||   0 |   0 |   0 ||   0 |   0 |  0 |
    // |     |   1 |   4 |   5 ||   7 |   5 |   0 ||   0 |   0 |  0 |
    // |     |     |     |     ||     |   1 |   2 ||   5 |   0 |  0 |
    
    float f = mhz * 1000; // MHz to KHz

    if(khz == 0) {
        khz = 12.5; // 12.5K

        if(bw_ == 1) // 25K
            khz = 25;
    }

    if(dir) f = f + khz;
    else f = f - khz;

    f = f / 1000;

    if(scan(f)) return f;
    else return loopScan(f, dir);
}

float SA818Controller::next(float mhz, float khz) {
    return loopScan(mhz, true, khz);
}

float SA818Controller::previous(float mhz, float khz) {
    return loopScan(mhz, false, khz);
}
