import os
import numpy as np
import time


S = 21

def save_to_file(file_name, t):
    f = open(file_name, "w")
    f.write(t)
    f.close()

def get_index(x,y):
    return y*S+x

def get_x(index):
    return index%S

def get_y(index):
    return index//S


class Ship:
    def __init__(self, index, cargo, player_id, name):
        self.x = get_x(index)
        self.y = get_y(index)
        self.cargo = cargo
        self.player_id = player_id
        self.name = name


class Shipyard:
    def __init__(self, index, player_id, name):
        self.x = get_x(index)
        self.y = get_y(index)
        self.player_id = player_id
        self.name = name

def create_halite_board(obs):
    halite_board = np.zeros((S,S))
    for x in range(S):
        for y in range(S):
            halite_board[x][y] = obs['halite'][get_index(x,y)]
    return halite_board

def get_my_id(obs):
    return obs['player']

def get_turn_number(obs):
    return obs["step"]

def get_players_halite(obs, player_id):
    return obs['players'][player_id][0]

def get_players_ships(obs, player_id):
    ships = []
    ship_info = obs['players'][player_id][2]
    for name in ship_info:
        index = ship_info[name][0]
        cargo = ship_info[name][1]
        ships.append(Ship(index, cargo, player_id, name))
    return ships

def get_players_shipyards(obs, player_id):
    shipyards = []
    shipyard_info = obs['players'][player_id][1]
    for name in shipyard_info:
        index = shipyard_info[name]
        shipyards.append(Shipyard(index, player_id, name))
    return shipyards


def generate_input_from_obs(obs, file_name):

    f = open(file_name, "w")
    f.write("1\n")
    halite = create_halite_board(obs)
    for x in range(S):
        for y in range(S):
            f.write(str(halite[x][y])+" ")
        f.write("\n")
    turn_n = get_turn_number(obs)
    f.write(str(turn_n)+"\n")
    my_id = get_my_id(obs)
    f.write(str(my_id)+"\n")

    for pid in range(4):
        h = get_players_halite(obs, pid)
        f.write(str(h)+"\n")
        ships = get_players_ships(obs, pid)
        f.write(str(len(ships))+"\n")
        for ship in ships:
            f.write(ship.name+" ")
            f.write(str(ship.x)+" "+str(ship.y)+" "+str(ship.cargo)+"\n")

        shipyards = get_players_shipyards(obs, pid)
        f.write(str(len(shipyards))+"\n")
        for shipyard in shipyards:
            f.write(shipyard.name+" ")
            f.write(str(shipyard.x)+" "+str(shipyard.y)+"\n")
    f.close()

def read_answer(file_name):
    f = open(file_name)
    l = f.readlines()
    f.close()
    actions = {}
    for line in l:
        line = line.strip()
        if len(line)!=0:
            name, action = line.split()
            if action != "-":
                actions[name]=action
    return actions


def agent(obs):
    in_file_name = "obs" + unique_id + ".in"
    out_file_name = "action" + unique_id + ".out" 
    generate_input_from_obs(obs, in_file_name)
    run_command = "./{0} < {1} > {2}".format(bin_file_name, in_file_name, out_file_name)
    os.system(run_command)
    actions = read_answer(out_file_name)
    return actions



cpp_program = '''
//#include <bits/stdc++.h>
#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <cassert>
#include <iomanip>


using namespace std;

#define sim template < class c
#define ris return * this
#define dor > debug & operator <<
#define eni(x) sim > typename   enable_if<sizeof dud<c>(0) x 1, debug&>::type operator<<(c i) {
sim > struct rge { c b, e; };
sim > rge<c> range(c i, c j) { return rge<c>{i, j}; }
sim > auto dud(c* x) -> decltype(cerr << *x, 0);
sim > char dud(...);
struct debug {
~debug() { cerr << endl; }
eni(!=) cerr << boolalpha << i; ris; }
eni(==) ris << range(begin(i), end(i)); }
sim, class b dor(pair < b, c > d) {
  ris << "(" << d.first << ", " << d.second << ")";
}
sim dor(rge<c> d) {
  *this << "[";
  for (auto it = d.b; it != d.e; ++it)
    *this << ", " + 2 * (it == d.b) << *it;
  ris << "]";
}
};
#define var(...) " [" << #__VA_ARGS__ ": " << (__VA_ARGS__) << "] "

#define pb push_back
#define mp make_pair
#define st first
#define nd second
#define ALL(x) (x).begin(), (x).end()
#define SZ(x) ((int)(x).size())
#define un(x) x.erase(unique(ALL(x)),x.end())
#define REP(i,x,v)for(int i=x;i<=v;i++)
#define REPD(i,x,v)for(int i=x;i>=v;i--)
#define FOR(i,v)for(int i=0;i<v;i++)
#define TRAV(i,v)for(auto &i:(v))
#define REMIN(x,v) x=min(x,v);
#define REMAX(x,v) x=max(x,v);

mt19937_64 rng(134569);
int rd(int l, int r) {return uniform_int_distribution<int>(l, r)(rng);}

using ll=long long;
using pii=pair<int,int>;
using pll=pair<ll,ll>;
using vi=vector<int>;
using vd=vector<double>;
using vll=vector<ll>;
const int MOD=1e9+7;//998244353;

const int INF = (1<<29);
const int S = 21;


double haliteBoard[S][S];
double haliteBoardCopy[S][S];

double playersHalite[4];

int shipsNo[4];
vector<string> shipsName[4];
vector<pair<int,int> > shipsPos[4];
vector<double> shipsCargo[4];

int shipyardsNo[4];
vector<string> shipyardsName[4];
vector<pair<int,int> > shipyardsPos[4];

int dx[5] = {0, 0, -1, 1, 0};
int dy[5] = {-1, 1, 0, 0, 0};
string dirNames[5] = {"NORTH", "SOUTH", "WEST", "EAST", "-"};

vector<vector<double>> bestPathScoresForShipsInGen;


int turnNo;
int myId;

double enemyCollisionBoard[S][S];
int distBoard[S][S];
int selfCollisionBoard[S][S];

bool testRun;


void read_obs() {
    for (int x=0;x<S;x++) {
        for (int y=0;y<S;y++) {
            cin>>haliteBoard[x][y];
        }
    }
    cin>>turnNo;
    cin>>myId;

    for (int pid=0;pid<4;pid++) {
        cin>>playersHalite[pid];
        int shipNo;
        cin>>shipNo;
        shipsNo[pid] = shipNo;
        //debug()<<var(pid)<<var(shipNo);
        shipsName[pid].clear();
        shipsPos[pid].clear();
        shipsCargo[pid].clear();

        for (int i = 0; i < shipNo; i++) {
            string shipName;
            int sx, sy;
            double cargo;
            cin >> shipName;
            cin >> sx;
            cin >> sy;
            cin >> cargo;
            //to samo co cin>>shipName>>sx>>sy>>cargo;
            shipsName[pid].push_back(shipName);
            shipsPos[pid].push_back(make_pair(sx, sy));
            //to samo (C++11) shipPos[pid].push_back({sx,sy});
            shipsCargo[pid].push_back(cargo);

        }

        //debug()<<var(pid)<<var(shipNames[pid]);

        shipyardsName[pid].clear();
        shipyardsPos[pid].clear();

        int shipyardNo;
        cin>>shipyardNo;
        shipyardsNo[pid] = shipyardNo;
        for (int i = 0; i < shipyardNo; i++) {
            string shipyardName;
            int sx, sy;
            cin >> shipyardName;
            cin >> sx;
            cin >> sy;
            shipyardsName[pid].push_back(shipyardName);
            shipyardsPos[pid].push_back(make_pair(sx, sy));
        }
    }
}



void createHaliteBoardCopy(){
    for(int x = 0; x<S; x++){
        for(int y = 0; y<S; y++){
            haliteBoardCopy[x][y] = haliteBoard[x][y];
        }
    }
    return;
}

pair<int, int> newPos(int x, int y, int d){
    x = (x+dx[d]+S)%S;
    y = (y+dy[d]+S)%S;
    return make_pair(x,y);
}


struct Ship{
    double cargo;
    int x,y,playerId;
    string name;

    Ship(string _name, int _sx, int _sy, double _cargo, int _playerId) {
        x=_sx;
        y=_sy;
        cargo = _cargo;
        name = _name;
        playerId = _playerId;
    }
};

struct Shipyard{
    int x,y,playerId;
    string name;

    Shipyard(string _name, int _sx, int _sy, int _playerId) {
        x=_sx;
        y=_sy;
        name = _name;
        playerId = _playerId;
    }
};

vector<Ship> getPlayerShips(int pId){
    vector<Ship> pShips;
    for ( int i=0 ; i < shipsNo[pId]; i++ ){
        Ship s(shipsName[pId][i], shipsPos[pId][i].first, shipsPos[pId][i].second, shipsCargo[pId][i], pId );
        pShips.push_back(s);
    }
    shuffle(pShips.begin(), pShips.end(), rng);
    return pShips;
}

vector<Shipyard> getPlayerShipyards(int pId){
    vector<Shipyard> pShipyards;
    for ( int i=0 ; i < shipyardsNo[pId]; i++ ){
        Shipyard s(shipyardsName[pId][i], shipyardsPos[pId][i].first, shipyardsPos[pId][i].second, pId );
        pShipyards.push_back(s);
    }
    shuffle(pShipyards.begin(), pShipyards.end(), rng);
    return pShipyards;
}

struct Player{
    int playerId;
    double halite;
    vector<Ship> ships;
    vector<Shipyard> shipyards;
    int nShips;
    int nShipyards;

    Player(){}

    Player(int _pId){ //, vector<Ship> _ships,vector<Shipyard> _shipyards ){
        playerId = _pId;
        halite = playersHalite[_pId];
        ships = getPlayerShips(_pId);
        shipyards = getPlayerShipyards(_pId);
        nShips = shipsNo[_pId];
        nShipyards = shipyardsNo[_pId];
    }

};



Player marta;
int myHalite;
vector<Ship> myShips;
vector<Shipyard> myShipyards;





void createSelfCollisionBoard(){
    for (int x=0; x<S; x++){
        for (int y=0; y<S; y++){
            selfCollisionBoard[x][y] = 0;
        }
    }
}

vector<int> getEnemiesId(){
    vector<int> enemiesId;
    for (int i=0; i<4 ; i++){
        if (i!=myId){
            enemiesId.push_back(i);
        }
    }
    assert(enemiesId.size() == 3);
    return enemiesId;
}

vector<Player> getEnemies(){
    vector<Player> enemies;
    vector<int> enemiesId = getEnemiesId();
    for (int i=0; i<3; i++){
        int id = enemiesId[i];
        Player enemy(id);
        enemies.push_back(enemy);
    }
    return enemies;
}


void createEnemyCollisionBoard(){
    for (int x=0; x<S; x++){
        for( int y=0; y<S; y++){
            enemyCollisionBoard[x][y] = INF;
        }
    }
    vector<Player> enemies = getEnemies();


    for (int e=0; e<3; e++){
        Player enemy = enemies[e];
        //debug()<<var(e)<<var(dirNames[4]);
        for(int s=0; s< enemy.nShips; s++ ){
            Ship ship = enemy.ships[s];
            for (int d=0; d<5; d++ ){
                pair<int, int> pos = newPos(ship.x, ship.y, d);
                enemyCollisionBoard[pos.first][ pos.second] = min(ship.cargo, enemyCollisionBoard[pos.first][ pos.second]);
            }
        }
        //debug()<<var(dirNames[4]);
        for (int sh=0; sh< enemy.nShipyards; sh++){
            Shipyard shipyard = enemy.shipyards[sh];
            enemyCollisionBoard[shipyard.x][shipyard.y] = -1;
        }
    }
}

int distance(int x1, int y1, int x2, int y2){
    int dxx = abs(x1-x2);
    int dyy = abs(y1-y2);
    return min(dxx, S-dxx) + min(dyy, S-dyy);
}

int distClosestShipyard(int x, int y, vector<Shipyard> shipyards){
    int ns = shipyards.size();
    int minDist = 2*S;
    for( int i=0; i<ns; i++){
        Shipyard s = shipyards[i];
        int currentDist = distance(x, y, s.x, s.y);
        minDist = min(minDist, currentDist);
    }
    return minDist;
}

double scorePath(int shipX, int shipY, vector<int> path){
    double pathScore = 0;
    int nx = shipX;
    int ny = shipY;
    int pLen = path.size();
    double weight = 1;
    for (int i=0; i<pLen; i++){
        int d = path[i];
        pair<int, int> nPos = newPos(nx, ny, d);
        nx = nPos.first;
        ny = nPos.second;
        if (d==4){
            pathScore += 0.25*haliteBoardCopy[nx][ny]*weight;
            haliteBoardCopy[nx][ny] -= 0.25 * haliteBoardCopy[nx][ny];
        }
        weight *= 0.97;
    }
    int shyDist = distBoard[nx][ny];
    pathScore /= (shyDist+pLen);

    nx = shipX;
    ny = shipY;
    for (int i=0; i<pLen; i++){
        int d = path[i];
        pair<int, int> nPos = newPos(nx, ny, d);
        nx = nPos.first;
        ny = nPos.second;
        if (d==4){
            haliteBoardCopy[nx][ny] = haliteBoard[nx][ny];
        }
    }
    return pathScore;
}

vector<int> extendPathRandomly(vector<int> path, int addLen){
    int forbidDir[5] = {1, 0, 3, 2, -1};
    int pathLen = path.size();
    int forbD = -1;
    if (pathLen != 0){
        forbD = forbidDir[path[pathLen-1]];
    }
    for (int i =0; i<addLen; i++){
        vector<int> moves;
        for (int d=0; d<5; d++){
            if (d!= forbD) {
                moves.push_back(d);
            }
        }
        int lastMovesIdx = moves.size()-1;
        int d = moves[rd(0,lastMovesIdx)];
        path.push_back(d);
        forbD = forbidDir[d];
    }
    return path;
}

vector<int> randomPath(int pathLen, vector<int> safeMoves){
    int safeMovesNo = safeMoves.size();
    vector<int> path;
    path.push_back(safeMoves[rd(0, safeMovesNo-1)]);
    path = extendPathRandomly(path, pathLen-1);
    return path;
}

struct Individual{
    vector<int> path;
    int len;
    double score;

    Individual(vector<int> _path, double _score){
        path = _path;
        len = _path.size();
        score = _score;
    }

    bool operator < (const Individual& indiv2) const
    {
        return (score < indiv2.score);
    }

    void mutateSuffix(){
        int cut = rd(1, len-1);
        int suffixLen = len - cut;
        while(SZ(path) > cut){
            path.pop_back();
        }
        path = extendPathRandomly(path, suffixLen);
        score = -1;
        assert(SZ(path)==len);
    }

    void mutateSection(){
        int firstIdx = rd(1,len-1);
        int lastIdx = firstIdx;
        while (firstIdx == lastIdx){
            lastIdx = rd(1, len-1);
        }
        if (firstIdx > lastIdx){
            int swapIdx = firstIdx;
            firstIdx = lastIdx;
            lastIdx = swapIdx;
        }
        //debug()<<var(firstIdx)<<var(lastIdx);
        int vectX = 0;
        int vectY = 0;
        for (int i = firstIdx; i<lastIdx+1; i++){
            vectX += dx[path[i]];
            vectY += dy[path[i]];
        }
        int sectionLen = lastIdx - firstIdx +1;
        vector<int> section;
        vector<int> pathTail;
        for ( int i = lastIdx + 1; i<len; i++){
            pathTail.push_back(path[i]);
        }
        while ( SZ(path) > firstIdx){
            path.pop_back();
        }
        for (int i = 0; i < abs(vectX); i++){
            if (vectX<0){
                section.push_back(2);
            }
            if (vectX > 0){
                section.push_back(3);
            }
        }
        
        for (int i = 0; i < abs(vectY); i++){
            if (vectY < 0){
                section.push_back(0);
            }
            if (vectY > 0){
                section.push_back(1);
            }
        }
        while( SZ(section)< sectionLen){
            section.push_back(4);
        }
        shuffle(section.begin(), section.end(), rng);
        for (int i=0; i<sectionLen; i++){
            path.push_back(section[i]);
        }
        int tailLen = pathTail.size();
        for (int i = 0; i< tailLen; i++){
            path.push_back(pathTail[i]);
        }
        score = -1;
    }

    void mutate(){
        int mutation = rd(0,1);
        if (mutation==0){
            mutateSuffix();
        }
        else{
            mutateSection();
        }
        assert(SZ(path) == len);
    }
};

vector<Individual> evolve(vector<Individual> generation, int shipX, int shipY, vd& bestPathScores){
    sort(generation.begin(), generation.end());
    int genN = generation.size();
    if (testRun==1){
        bestPathScores.push_back(generation[genN-1].score);
    }
    int topNo = 0.1 * genN;
    vector<Individual> bestIndivs;
    for (int i = genN - topNo; i < genN; i++){
        bestIndivs.push_back(generation[i]);
    }
    vector<Individual> newGeneration = bestIndivs;
    vector<int> indivsWeights;
    for (int i=0; i< topNo; i++){
        indivsWeights.push_back(i+1);
    }
    while( SZ(newGeneration)< genN){
        int idx = rd(0, topNo-1);
        Individual indiv = bestIndivs[idx];
        indiv.mutate();
        indiv.score = scorePath(shipX, shipY, indiv.path);
        newGeneration.push_back(indiv);
    }
    assert(SZ(newGeneration) == genN);
    return newGeneration;
}

void substractMinedHalite(int shipX, int shipY, vector<int> path){
    int nx = shipX;
    int ny = shipY;
    int pathLen = path.size();
    for (int i = 0; i<pathLen; i++){
        int d = path[i];
        pair<int, int> nPos = newPos(nx, ny, d);
        nx = nPos.first;
        ny = nPos.second;
        if (d==4){
            haliteBoard[nx][ny] -= 0.25*haliteBoard[nx][ny];
        }
    }
    return;
}



int genNo = 1500;
int evolNo =10;

int findBestMove(int shipX, int ShipY, vector<int> safeMoves, vd& bestPathScores){
    vector<Individual> generation;
    int pLen = 10;
    for (int i = 0; i < genNo; i++){
        vector<int> path = randomPath(pLen, safeMoves);
        assert(SZ(path) == pLen);
        double score = scorePath(shipX, ShipY, path);
        Individual indiv(path, score);
        generation.push_back(indiv);
    }

    for (int i = 0; i<evolNo; i++){
        generation = evolve(generation, shipX, ShipY, bestPathScores);
        assert(SZ(generation) == genNo);
    }

    sort(generation.begin(), generation.end());
    Individual bestIndiv = generation[genNo-1];
    substractMinedHalite(shipX, ShipY, bestIndiv.path);
    if (testRun==1){
        bestPathScores.push_back(bestIndiv.score);
    }
    return bestIndiv.path[0];
}

vector<int> findSafeMoves(Ship ship){
    vector<int> safeMoves;
    for (int d=0; d<5; d++){
        pair<int, int> nPos = newPos(ship.x, ship.y, d);
        int nx = nPos.first;
        int ny = nPos.second;
        if( enemyCollisionBoard[nx][ny] <= ship.cargo || selfCollisionBoard[nx][ny] == 1){
            continue;
        }
        safeMoves.push_back(d);
    }
    return safeMoves;
}

vector<int> findEnemyShysAvoidingMoves(Ship ship){
    vector<int> moves;
    for (int d=0; d<5; d++){
        pair<int, int> nPos = newPos(ship.x, ship.y, d);
        int nx = nPos.first;
        int ny = nPos.second;
        if( enemyCollisionBoard[nx][ny]  == -1){
            continue;
        }
        moves.push_back(d);
    }
    return moves;
}

bool isEnoughHaliteOnBoard(){
    int nS = marta.nShips;
    if (nS ==0){
        return true;
    }
    double sumOfHalite = 0;
    for (int i=0; i< nS; i++){
        Ship s = myShips[i];
        //debug() << var(s.x) << var(s.y);
        sumOfHalite += 0.25*haliteBoard[s.x][s.y];
    }
    double averHalite = sumOfHalite / nS;
    if ( averHalite *(400 - turnNo)>2000){
        return true;
    }
    return false;
}

bool buildShipDecide(int myShipsNo, int myShipyardsNo){
    if(myHalite >= 500 && myShipsNo < myShipyardsNo*7 && isEnoughHaliteOnBoard()==true){
        return true;
    }
    return false;
}

bool buildShipyardDecide(int distClosestShy, int myShipyardsNo ){
    int targetShipyards = 3;
    if (myHalite >=500 && distClosestShy > 4 && myShipyardsNo < targetShipyards ){
        return true;
    }
    return false;
}



vector<pair<string, string>> agent(){
    vector<pair<string, string>> actions;
    int turnsLeft = 400 - turnNo;


    createEnemyCollisionBoard();
    createSelfCollisionBoard();
    createHaliteBoardCopy();


    marta = Player(myId);
    myHalite = marta.halite;
    myShips = marta.ships;
    myShipyards = marta.shipyards;

    int myShipsNo = marta.nShips;
    int myShipyardsNo = marta.nShipyards;


    int cargoTreshold = 300;
    //debug() <<  var(myShipyardsNo) << var(marta.nShipyards) << var(myShipyards.size());
    for (int i =0; i< marta.nShipyards; i++){
            //debug()<< var(i);
        Shipyard shy = myShipyards[i];
        //debug()<<var(buildShipDecide());
        if (buildShipDecide(myShipsNo, myShipyardsNo)==true){
            actions.push_back(make_pair(shy.name, "SPAWN"));
            myShipsNo += 1;
            myHalite -= 500;
            //debug() << var(shy.x) << var(shy.y);
            selfCollisionBoard[shy.x][shy.y] = 1;
        } else {
            actions.push_back(make_pair(shy.name, "-"));
        }
    }

    //debug() << var(actions)<< var(shipsNo) ;
    for (int i=0; i< marta.nShips ; i++){
        //cout << "weslo w petle" << endl;
        Ship s = myShips[i];

        int distClosestShy = distClosestShipyard(s.x, s.y, myShipyards);

        if (buildShipyardDecide(distClosestShy, myShipyardsNo)==true){
            actions.push_back(make_pair(s.name, "CONVERT"));
            myShipyardsNo += 1;
            myHalite -= 500;
            continue;
        }
        vector<int> safeMoves = findSafeMoves(s);
        int safeMovesNo = safeMoves.size();
        if (safeMovesNo == 0){
            safeMoves = {0,1,2,3,4};
            //selfCollisionBoard[s.x][s.y]=1;
            //actions.push_back(make_pair(s.name, "-"));
            //continue;
        }

        vector<int> returnMoves;
        int returnMovesNo = returnMoves.size();
        int bestMove;
        bool returnCondition = s.cargo > cargoTreshold || (s.cargo>50 && turnsLeft < distClosestShy + 5);

        if (returnCondition == true){
            for(int j =0; j<safeMovesNo; j++){
                int d = safeMoves[j];
                pair<int, int> nPos = newPos(s.x, s.y, d);
                int newDistClosestShy = distClosestShipyard(nPos.first, nPos.second, myShipyards);
                if (newDistClosestShy < distClosestShy){
                    returnMoves.push_back(d);
                }
            }
            returnMovesNo = returnMoves.size();
            if (returnMovesNo != 0){
                bestMove = returnMoves[rd(0, returnMovesNo-1)];
                //cout << s.name << " " << bestMove << endl;
            }
        }
        //debug()<< var(returnCondition)<< var(returnMovesNo);
        //debug() << var(s.name)<< var(s.cargo);
        if (returnCondition==false || returnMovesNo == 0 ){

            vd bestPathScores;
            bestMove = findBestMove(s.x, s.y, safeMoves, bestPathScores);
            //cout << s.name << " " << bestMove << " "<< dirNames[bestMove]<< endl;
            if (testRun== 1){
                bestPathScoresForShipsInGen.push_back(bestPathScores);
                //debug() << var(SZ(bestPathScores));
            }
        }

        actions.push_back(make_pair(s.name, dirNames[bestMove]));
        pair<int, int> nPos = newPos(s.x, s.y, bestMove);
        int nx = nPos.first;
        int ny = nPos.second;
        selfCollisionBoard[nx][ny] = 1;

        //debug() << var(actions);
    }
    //debug() << var(actions)<< var(shipsNo) ;
    return actions;
}


void writeOutPathsScores(){
    int nS = bestPathScoresForShipsInGen.size();
    debug() << var(nS);
    for (int i=0; i< nS; i++){
        vd bestPathScores = bestPathScoresForShipsInGen[i];
        int ngen = bestPathScores.size();
        //double score = bestPathScores[0];
        //assert(ngen==evolNo) ;
        for (int j=0; j < ngen; j++){
            //double currentScore = bestPathScores[j];
            //if (currentScore < score){
            //  cout << "jest dramat" <<endl;
            //}
            cout << bestPathScores[j] << " ";
            //score = currentScore;
        }
        cout << endl;
    }
}


vd getAverages(vector<vector<double>> shipsDataInGen){
    vd averages;
    int shipNo = SZ(shipsDataInGen);
    if (shipNo==0){
        return averages;
    }
    for (int j=0; j < SZ(shipsDataInGen[0]); j++){
        double average = 0;
        for( int i=0; i<shipNo; i++){
            average += shipsDataInGen[i][j];
        }
        average /= shipNo;
        averages.push_back(average);
    }
    return averages;
}

vd getStds(vector<vd> shipsDataInGen){
    vd stds;
    int shipNo = SZ(shipsDataInGen);
    if (shipNo==0){
        return stds;
    }
    vd averages = getAverages(shipsDataInGen);
    for (int j=0; j < SZ(shipsDataInGen[0]); j++){
        double std = 0;
        for( int i=0; i<shipNo; i++){
            std += (averages[j] - shipsDataInGen[i][j]) * (averages[j] - shipsDataInGen[i][j]);
        }
        std /= shipNo;
        std = sqrt(std);
        stds.push_back(std);
    }
    return stds;
}

vector<vd> getScoreProgressWrtAnotherScore(bool previous){  // if previous =0 (1), compute score progress wrt the first (previous) score
    vector<vd> scoreProgress;
    int shipNo = SZ(bestPathScoresForShipsInGen);
    if (shipNo==0){
        return scoreProgress;
    }
    int nOfGen = SZ(bestPathScoresForShipsInGen[0]);
    for (int i=0; i< shipNo; i++){
        vd currentScores = bestPathScoresForShipsInGen[i];
        vd progressForShip;
        double referenceScore = currentScores[0];
        for(int j=1; j< nOfGen; j++){
            progressForShip.push_back(currentScores[j]- referenceScore);
            if (previous==1){
                referenceScore = currentScores[j];
            }
        }
        scoreProgress.push_back(progressForShip);
    }
    return scoreProgress;
}


double getAverage(vd vec){
    double ave = 0;
    //if (SZ(vec)==0){
    //  return ave;
    //}
    for(int i =0; i< SZ(vec); i++){
        ave += vec[i];
    }
    ave /= SZ(vec);
    return ave;
}

double getStd(vd vec){
    double ave = getAverage(vec);
    double stDev=0 ;
    assert(SZ(vec) !=0);
    //if (SZ(vec)==0){
    //  return stDev;
    //}
    for(int i=0; i< SZ(vec); i++){
        stDev += (ave - vec[i]) * (ave - vec[i]);
    }
    stDev /= SZ(vec);
    stDev = sqrt(stDev);
    return stDev;
}

vd getFinalBestPathScores(){
    vd finalScores;
    int shipNo = SZ(bestPathScoresForShipsInGen);
    if (shipNo==0){
        return finalScores;
    }
    int nOfGen = SZ(bestPathScoresForShipsInGen[0]);
    for (int i=0; i< shipNo; i++){
        finalScores.push_back(bestPathScoresForShipsInGen[i][nOfGen-1]);
    }
    return finalScores;
}


int main()
{   
    //ios_base::sync_with_stdio(0);cin.tie(0);
    cout << fixed << setprecision(5);
    cerr << fixed << setprecision(5);

    int obsNo;
    cin >> obsNo;
    if (obsNo==1){
        testRun = false;
        read_obs();
        auto actions = agent();
        //debug()<< var(SZ(actions));
        for(auto a: actions) {
            cout<<a.first << " " << a.second << endl;
        }

    } else{
        vd aveBestScoresInObss;
        vd stdBestScoresInObss;
        testRun = true;
        for(int i =0; i<obsNo; i++){
            read_obs();
            auto actions = agent();
            //sort(actions.begin(), actions.end());
            //debug()<< var(SZ(actions));
            //for(auto a: actions) {
            //    cout << a.first << " " << a.second<<endl;
            //}
            vd finalScores = getFinalBestPathScores();
            if (SZ(finalScores)==0){
                continue;
            }
            //debug() << var(getAverages(bestPathScoresForShipsInGen));
            //debug() << var(getStds(bestPathScoresForShipsInGen));
            //vector<vd> scoresWrtFirstScore = getScoreProgressWrtAnotherScore(0);
            //vector<vd> scoresWrtPreviousScore = getScoreProgressWrtAnotherScore(1);
            //debug() << var(getAverages(scoresWrtFirstScore));
            //debug() << var(getStds(scoresWrtFirstScore));
            //debug() << var(getAverages(scoresWrtPreviousScore));
            //debug() << var(getStds(scoresWrtPreviousScore));
            //cout << endl;
            bestPathScoresForShipsInGen.clear();
            double aveBestScoreInObs = getAverage(finalScores);
            double stdBestScoreInObs = getStd(finalScores);
            aveBestScoresInObss.push_back(aveBestScoreInObs);
            stdBestScoresInObss.push_back(stdBestScoreInObs);
        }
        debug() << var(obsNo);
        double aveBestScore = getAverage(aveBestScoresInObss);
        double stdOfAveBestScoresInObs = getStd(aveBestScoresInObss);
        //double aveOfStdsOfScoresInObs = getAverage(stdBestScoresInObss);
        debug()<< var(genNo)<<var(evolNo);
        debug()<< var(aveBestScore) << var(stdOfAveBestScoresInObs); // << var(aveOfStdsOfScoresInObs);
    }
    
    return 0;
}








'''
unique_id = str(time.time_ns())
cpp_file_name = "created"+unique_id+".cpp"
bin_file_name = "created"+unique_id
save_to_file(cpp_file_name, cpp_program)
compile_command = "g++ {0} -O2 -o {1}".format(cpp_file_name, bin_file_name)
os.system(compile_command)

#ex_obs = {'halite': [0.0, 0.0, 23.908, 0.0, 0.0, 34.647, 75.626, 0.0, 0.0, 25.891, 0.0, 35.453, 0.0, 0.0, 26.248, 33.968, 0.0, 0.0, 143.217, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 38.216, 0.0, 37.347, 0.0, 0.0, 0.0, 49.026, 0.0, 43.259, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 25.363, 0.0, 0.0, 42.332, 51.161, 69.692, 40.731, 49.333, 0.0, 44.803, 0.0, 47.265, 33.817, 52.918000000000006, 0.0, 0.0, 304.912, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 43.237, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 27.031999999999996, 0.0, 14.983, 0.0, 0.0, 0.0, 27.189, 0.0, 33.749, 0.0, 24.741, 0.0, 0.0, 0.0, 84.468, 0.0, 133.142, 0.0, 0.0, 0.0, 0.0, 37.501, 0.0, 23.09, 0.0, 12.208, 0.0, 17.057, 0.0, 0.0, 0.0, 21.442, 0.0, 35.590999999999994, 0.0, 77.977, 0.0, 179.69, 0.0, 0.0, 85.756, 0.0, 54.868, 0.0, 0.0, 21.715, 25.134, 15.912, 17.128, 0.0, 18.767, 0.0, 48.294, 43.038, 75.759, 56.31399999999999, 0.0, 0.0, 73.498, 0.0, 159.32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 35.637, 0.0, 0.0, 0.0, 20.649, 0.0, 0.0, 0.0, 60.982, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 79.304, 0.0, 86.46, 0.0, 0.0, 0.0, 0.0, 25.973, 0.0, 0.0, 36.453, 0.0, 0.0, 25.973, 0.0, 0.0, 0.0, 0.0, 37.09, 0.0, 42.953, 0.0, 85.603, 78.66, 49.923, 0.0, 0.0, 34.647, 0.0, 0.0, 33.968, 0.0, 34.647, 0.0, 0.0, 34.647, 0.0, 0.0, 48.944, 31.943, 36.778, 0.0, 0.0, 91.398, 0.0, 67.458, 0.0, 0.0, 0.0, 8.659, 65.517, 57.421, 0.0, 143.221, 0.0, 8.659, 0.0, 0.0, 0.0, 32.264, 0.0, 42.464, 0.0, 0.0, 82.79, 99.101, 44.33, 0.0, 0.0, 34.647, 0.0, 0.0, 32.649, 0.0, 32.009, 0.0, 0.0, 34.647, 0.0, 0.0, 16.285, 23.557, 20.713, 0.0, 39.927, 0.0, 91.898, 0.0, 0.0, 0.0, 0.0, 25.464, 0.0, 0.0, 121.519, 0.0, 0.0, 25.973, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 30.571, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 82.502, 0.0, 0.0, 0.0, 48.143, 0.0, 0.0, 0.0, 16.428, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 85.08, 0.0, 85.841, 0.0, 0.0, 88.766, 94.186, 91.355, 0.0, 0.0, 0.0, 0.0, 32.384, 27.206, 18.933, 33.31, 0.0, 0.0, 22.014, 0.0, 25.538, 0.0, 0.0, 94.173, 0.0, 97.442, 0.0, 100.232, 0.0, 57.898, 0.0, 0.0, 0.0, 20.525, 0.0, 19.776, 0.0, 27.785, 0.0, 36.707, 0.0, 0.0, 0.0, 0.0, 70.65, 0.0, 99.345, 0.0, 0.0, 0.0, 24.586, 0.0, 20.334, 0.0, 18.918, 0.0, 0.0, 0.0, 33.354, 0.0, 38.908, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 22.422, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 30.067, 0.0, 0.0, 28.044, 37.294, 37.568, 53.02, 49.583, 0.0, 29.839, 22.739, 23.2, 22.063, 41.919, 0.0, 0.0, 106.614, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 38.672, 0.0, 50.845, 0.0, 0.0, 0.0, 20.754, 0.0, 13.53, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 21.515, 0.0, 0.0, 30.766, 73.518, 0.0, 0.0, 34.189, 0.0, 32.41, 0.0, 0.0, 0.0, 30.163, 0.0, 0.0, 95.107, 0.0, 0.0], 'player': 2, 'players': [[3277, {'1-1': 110}, {'10-1': [126, 81], '5-1': [140, 110], '7-1': [405, 289], '76-1': [86, 193], '77-1': [406, 305], '79-1': [218, 499], '82-1': [137, 197], '85-1': [73, 180]}], [5657, {'1-2': 120}, {'66-1': [121, 361]}], [411, {'1-3': 320, '52-1': 222, '8-3': 302}, {'38-1': [211, 27], '41-1': [170, 112], '47-1': [233, 100], '5-2': [340, 0], '53-1': [168, 446], '7-2': [169, 84], '72-1': [150, 27], '78-1': [189, 182], '82-2': [296, 27], '94-1': [254, 26]}], [3771, {'1-4': 330, '15-1': 369, '35-1': 434, '58-1': 270, '87-1': 54}, {'16-1': [144, 60], '16-2': [119, 255], '19-1': [352, 10], '2-4': [57, 17], '21-1': [373, 21], '30-1': [141, 306], '38-2': [75, 30], '38-3': [31, 307], '4-3': [308, 57], '45-1': [396, 179], '47-3': [76, 301], '51-1': [431, 50], '56-1': [323, 47], '60-1': [33, 0], '68-1': [81, 283], '71-1': [314, 15], '77-2': [208, 167], '80-2': [188, 176], '85-2': [229, 69], '88-1': [49, 289], '94-2': [18, 222], '94-3': [354, 88], '94-4': [60, 210], '94-5': [255, 101], '94-6': [117, 74]}]], 'step': 109}

#agent(ex_obs)










