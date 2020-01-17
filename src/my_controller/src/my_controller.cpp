//Struttura base di qualsiasi controllore
#include <controller_interface/controller.h> //interfaccia del controllore
#include <hardware_interface/joint_command_interface.h> //interfaccia dell'hardware
#include <pluginlib/class_list_macros.h> //interfaccia di plug-in. 

#include <std_msgs/Float64.h>//utilizzeremo questo per ricevere i GOALS MESSAGE (i messaggi obbiettivo) 

namespace my_controller_ns //diamo il nome al controllore così che non ci siano collisioni con altri controllori 
{//lo step successivo è creare la CLASSE del controllore
  
  class MyPositionController : 
  
  public controller_interface::Controller<hardware_interface::EffortJointInterface>
  {
    //il nome della classe sarà il TIPO di controllore che andremo a creare
    //controller_interface, la nostra classe EREDITA il tipo di controllore
    //EffortJointInterface è il TEMPLATE del controllore. Se volessimo generare un altro tipo di controllore, basterebbe cambiare questo template, specificandone uno diverso.
    //esempio se volessimo un controllore in POSIZIONE, VELOCITÀ. In quest caso abbiamo un effort joint controllor.
    //Perchè il plug-in lavori bene con il CONTROL MANAGER, dobbiamo scrivere all'interno della classe delle funzioni di "riconoscimento" : 
    //init, starting, updating, stopping
    
    //INIT = è la funzione che il CONTROL MANAGER chiama quando stà CARICANDO il controllore nel sistema
    bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n) //questa parte qua,sarà chiamata dal controller manager quando caricheremo il controller
    {
      //Questa prima parte serve a specificare i JOINTS
     std::string my_joint;
     if (!n.getParam("joint", my_joint))//n.getParam cerca il valore specifico "joint", di quel specifico GIUNTO.
     {
       ROS_ERROR("Could not find joint name");
       return false;       
     }
     
     joint_ = hw->getHandle(my_joint); //throws on failure //Una volta preso il PARAMETRO DEL GIUNTO, si va ad associare all'HARDWARE quel valore di giunto
     command_ = joint_.getPosition(); //set the current joint goal to the current joint getPosition //Una volta individuato il GIUNTO, si verifica l'hardware e successivamente si verifica la sua POSIZIONE.
     //Questa posizione viene mantenuta finchè non arriva un comando che specifica di cambiare posizione.
     
     
     //Load gain using gains set on parameter server 
     if (!n.getParam("gain",gain_)) //In questo caso, si cerca il PARAMETRO "GUADAGNO" del nostro CONTROLLORE. Quello che fa, in questo caso, controlla
       //la posizione corrente, posizione desiderata -> crea una variabile errore che è pari la differenza delle due posizioni e tramite un guadagno
       //moltiplica questo valore errore.
     {
       ROS_ERROR("Could not find the gain parameter value");
       return false;
     }
     
     //Start command subscriber //E' un SUBSCRIBER al TOPIC chiamato COMMAND. E' un tipo di comando, che questo nodo ascolterà per ricevere ordini. Ad esempio le POSIZIONI DESIDERATE.
     sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &MyPositionController::setCommandCB, this); //MyPositionController è una CALLBACK per dire che il comando è stato ricevuto 
     return true;
    }
  
  //Ora creiamo UPDATE LOOP, è il loop che effettua il lavoro del controllore. Se vogliamo, il controllore vero e prorio.Nell'immagine "ROS CONTROL",sarebbe il blocchettino giallo interno al
  //blocco Controller.
  //UPDATE sarà chiamato continuamente dal control manager con una frequenza molto alta.
  void update(const ros::Time& time, const ros::Duration& period)//Vediamo cosa fa il nostro "controllore"
  {
    double error = command_ - joint_.getPosition(); //l'errore è la differenza tra L'ULTIMO COMANDO RICEVUTO (command_) e la POSIZIONE ATTUALE del joint(joint_.getPosition)
    double commanded_effort = error*gain_;//successivamente si moltiplica l'errore per il GAIN PARAMETER, ottenuto precedentemente tramite init. (In pratica questa è la regola di controllo del Effort Joint, 
    //se avessimo usato la posizione o la velocita o un backstepping ecc... qui avremo scritto la legge del controllore)
    joint_.setCommand(commanded_effort); //questo comando serve a mandare il command effort appena calcolato ai vari giunti tramite il setCommand.
  }
  
  void setCommandCB(const std_msgs::Float64ConstPtr& msg)//questa è la callback per il topic che prendiamo dal sub_command_ (sopra).
  //Come vediamo il messaggio è un Float, il quale preleveremo la parte DATA di MSG e lo assegneremo a command
  {
   command_ = msg->data; 
  }
  
  //Ora vediamo lo START COMMAND. Ora come ora possiamo caricare un controller ma non possiamo Startarlo. Dunque usufruiremo di funzioni STARTING e STOPPING di modo che il control manager possa iniziare e 
  //fermare il controller
  void starting(const ros::Time& time) {}
  void stopping(const ros::Time& time) {}
  
  //Ora inseriamo anche delle funzioni o parametri PRIVATE, in quanto servono per la comunicazione interna tra le varie funzioni
private:
  hardware_interface::JointHandle joint_;//i giunti che vogliamo controllare devono essere di questo tipo.
  double gain_;//quello che ricaviamo come parametro e con il quale abbiamo moltiplicato l'errore
  double command_;//questo è il SET-POINT dove vogliamo che arrivino i giunti nel comando che daremo.
  ros::Subscriber sub_command_;//il subscriber è quello che ASCOLTA per i comandi.
  //Se avessimo un HEADER FILE, avremo questi 4 codici nel file.
  
  };
  
  //Infine dobbiamo includere un macro del PLUGINLIB, è molto importante in quanto permette di rendere questo controller reperibile al control manager. Come vediamo è un membro del namespace.
  PLUGINLIB_EXPORT_CLASS(my_controller_ns::MyPositionController, controller_interface::ControllerBase);//in pratica quello che fa è che dice al Control Manager: "Hey, abbiamo un nuovo controller che si 
  //chiama MyPositionController ed è del tipo controller_interface".
   
}

//Per compilare il file, dobbiamo utilizzare un file xml. Dunque creiamo il file xml, poi aggiungiamo in package.xml il codice in export per far riconoscere il plugin,
//e successivamente modifichiamo il CMakeList, aggiungendo la libreria. In quanto questo controllore è una NUOVA LIBRERIA.