function showUserList() {
  document.getElementById("userList").style.width = "250px";
}

function hideUserList() {
  document.getElementById("userList").style.width = "0";
}

function showLog() {
  document.getElementById("logView").style.width = "750px";
}

function hideLog() {
  document.getElementById("logView").style.width = "0";
}

function addUser(username) {
  const element = `<a id="user-${username}">${username}</a>`; 
  var cont = document.getElementById("ULCloseBtn");
  cont.insertAdjacentHTML("afterend", element);
}

function removeUser(username) {
  removeElement(`user-${username}`);
}

function addLog(text) {
  const element = `<a>${text}</a>`; 
  var cont = document.getElementById("logCloseBtn");
  cont.insertAdjacentHTML("afterend", element);
}
