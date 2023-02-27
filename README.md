# my_robot 〜自作パッケージでロボットを作る〜

<a id='0'></a>
## パッケージのビルド

　自分でロボットのパッケージを作って遊んでみる。ws ディレクトリとその中に src ディレクトリを作成し、
catkin_create_pkg コマンドで自分のパッケージを作成する。
```bash
catkin_create_pkg <pkg_name> <依存関係pkg> <依存関係pkg> ...
```
- <pkg_name> には自分の好きなパッケージの名前にする。ここでは my_robot という名前にした。
- <依存関係pkg> には、これから作成するパッケージに関わる他のパッケージまたはライブラリたちを指定する。
基本的に ROS 環境内で Python と c++ を使うには、rospy と roscpp というライブラリが必要となるので、必ずこれを記述しよう。
また、ROS のトピック通信のメッセージ種類として最も基本な std_msgs を今回は指定する。ほかにもメッセージの種類はたくさんある。

　パッケージ名を my_robot。初期に導入する依存関係たちは rospy、roscpp、std_msgs としてパッケージを作るとするならば、以下のコマンドでできる。
```bash
catkin_create_pkg my_robot rospy roscpp std_msgs
```
　すると、以下のようなログが表示される。以下のログの最後の行のような Successfully と書かれていれば自作パッケージの作成は完了である。
```bash
$ catkin_create_pkg my_robot rospy roscpp std_msgs
Created file my_robot/package.xml
Created file my_robot/CMakeLists.txt
Created folder my_robot/include/my_robot
Created folder my_robot/src
Successfully created files in /home/gai/catkin_ws/src/my_robot. Please adjust the values in package.xml.
```
　そしたら ws ディレクトリに移動して catkin build を実行する。
昔は catkin_make コマンドが主流だったが、これより改善された catkin build がおすすめである。
```bash
catkin build
```
　ビルドが成功したら、以下のコマンドでセットアップを通しておこう。
```bash
source devel/setup.bash
```
　そしたら、ROS のコマンドの一部である。 roscd コマンドを使って、先ほど作成した自作パッケージのディレクトリまで移動してみよう。
```bash
roscd my_robot
```
　作成した自作パッケージのディレクトリまで移動できたら ROS にも自作パッケージがちゃんと認識されていることとなる。

<a id='1'></a>
## パッケージ内を編集する

　ls コマンドで自作パッケージの中身を見てみよう。すると２つのファイルと２つのディレクトリが生成されているのがわかる。
```bash
$ ls
CMakeLists.txt  include  package.xml  src
```
　今はこれらのファイルに関しては無視。新たに２つディレクトリを作成する。今回作成するディレクトリは以下２つである。

- scripts : python スクリプトを記述するディレクトリ。
- msg : 自作メッセージを記述するディレクトリ。

　コマンドで２つのディレクトリを生成する。
```bash
mkdir scripts msg
```
　これで my_robot ディレクトリに２つの新しいディレクトリができた。

<a id='2'></a>
## パブリッシャーを作ってみよう
　早速パブリッシャーを作ってみよう。パブリッシャー（Publisher）とは、配信者という意味を持つ。配信者はその名の通りデータを配信する。
ここでは、整数を１秒間隔でカウントして送信するパブリッシャーを作ってみよう。
```
これから作るパブリッシャの挙動
0
1
2
3
4
.
.
.
```
　まずは名前を決めよう。これから作るパブリッシャーは整数を1秒間隔でカウントしながら配信するので int_publisher.py としよう。
好きはテキストエディタでファイルを作ろう。ここでは nano をつかってファイルを作成する。
```bash
nano scripts/int_publisher.py
```
　プログラムを書く準備ができたら、まずはシバンを記述しよう。
シバンとは、俺は Python 実行ファイルだぞ！と自己主張するための名札のようなものである。これを書かないと ROS が認識してくれない。
```python
#!/usr/bin/env python3
```
　そしたら必要なモジュールたちをインポートしよう。まずは python で ROS を扱うのに必須である rospy ライブラリをインポートする。
```python
import rospy
```
　次に、整数型のメッセージを扱うためのモジュールを呼び出す。これは std_msgs ライブラリ内にあるため、
以下のように記述して Int32 モジュールを呼び込む。
```python
from std_msgs.msg import Int32
```
　これで必要なモジュールのインポートは完了である。次に以下のプログラムでこの python プログラムをノードとして登録しよう。
ROS は１つの実行プログラムをノードという単位で管理する。そのためこのように rospy の init_node 関数を使ってノード名を設定する必要がある。
今回は整数でカウントしながら送信するノードを作るので、int_pub という名前のノードを作る。
```python
rospy.init_node('int_pub')
```
　次に、メインであるパブリッシュ（配信）設定をするプログラムを書く。パブリッシャーの設定は rospy の Publisher 関数でできる。
Publisher 関数は、以下3つの引数を必要とする。
```python
rospy.Publisher('トピック名', メッセージタイプ, キューサイズ)
```
　トピック名とは、電車で言うところの路線名である。メッセージタイプは電車で言うところの電車の名前である。キュー剤図はちょっとよくわからん。
例えをコロコロ変えてしまうが、例えば Publisher 関数の引数それぞれ以下のように設定したとしよう。
```python
rospy.Publisher('東海道新幹線', こだま, queue_size=1)
```
　これを要約すると、東海道新幹線というトピックにこだまというメッセージをキューサイズ１でパブリッシュすように設定するよ。である。
